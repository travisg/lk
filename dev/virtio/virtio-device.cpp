/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include "dev/virtio/virtio-device.h"

#include <lk/debug.h>
#include <lk/trace.h>
#include <lk/err.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <lk/pow2.h>
#include <lk/reg.h>
#include <arch/ops.h>
#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#include "virtio_priv.h"

#define LOCAL_TRACE 0

// TODO: switch to using reg.h mmio_ accessors
void virtio_device::virtio_reset_device() {
    mmio_config_->status = 0;
}

void virtio_device::virtio_status_acknowledge_driver() {
    mmio_config_->status |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
}

void virtio_device::virtio_status_driver_ok() {
    mmio_config_->status |= VIRTIO_STATUS_DRIVER_OK;
}

void virtio_device::virtio_set_guest_features(uint32_t word, uint32_t features) {
    mmio_config_->guest_features_sel = word;
    mmio_config_->guest_features = features;
}

uint32_t virtio_device::virtio_read_host_feature_word(uint32_t word) {
    mmio_config_->host_features_sel = word;
    return mmio_config_->host_features;
}

void virtio_device::virtio_free_desc(uint ring_index, uint16_t desc_index) {
    LTRACEF("dev %p ring %u index %u free_count %u\n", this, ring_index, desc_index, ring_[ring_index].free_count);

    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);
    DEBUG_ASSERT(desc_index < ring_len_[ring_index]);

    ring_[ring_index].desc[desc_index].next = ring_[ring_index].free_list;
    ring_[ring_index].free_list = desc_index;
    ring_[ring_index].free_count++;
}

uint16_t virtio_device::virtio_alloc_desc(uint ring_index) {
    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

    if (ring_[ring_index].free_count == 0)
        return 0xffff;

    DEBUG_ASSERT(ring_[ring_index].free_list != 0xffff);

    uint16_t i = ring_[ring_index].free_list;
    vring_desc *desc = &ring_[ring_index].desc[i];
    ring_[ring_index].free_list = desc->next;

    ring_[ring_index].free_count--;

    return i;
}

vring_desc *virtio_device::virtio_alloc_desc_chain(uint ring_index, size_t count, uint16_t *start_index) {
    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

    if (ring_[ring_index].free_count < count)
        return nullptr;

    /* start popping entries off the chain */
    vring_desc *last = nullptr;
    uint16_t last_index = 0;
    while (count > 0) {
        uint16_t i = ring_[ring_index].free_list;
        vring_desc *desc = &ring_[ring_index].desc[i];

        ring_[ring_index].free_list = desc->next;
        ring_[ring_index].free_count--;

        if (last) {
            desc->flags = VRING_DESC_F_NEXT;
            desc->next = last_index;
        } else {
            // first one
            desc->flags = 0;
            desc->next = 0;
        }
        last = desc;
        last_index = i;
        count--;
    }

    if (start_index)
        *start_index = last_index;

    return last;
}

void virtio_device::virtio_submit_chain(uint ring_index, uint16_t desc_index) {
    LTRACEF("dev %p, ring %u, desc %u\n", this, ring_index, desc_index);

    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);
    DEBUG_ASSERT(desc_index < ring_len_[ring_index]);

    /* add the chain to the available list */
    vring_avail *avail = ring_[ring_index].avail;

    avail->ring[avail->idx & ring_[ring_index].num_mask] = desc_index;
    mb();
    avail->idx++;

#if LOCAL_TRACE
    hexdump(avail, 16);
#endif
}

void virtio_device::virtio_kick(uint ring_index) {
    LTRACEF("dev %p, ring %u\n", this, ring_index);

    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

    mmio_config_->queue_notify = ring_index;
    mb();
}

status_t virtio_device::virtio_alloc_ring(uint index, uint16_t len) {
    LTRACEF("dev %p, index %u, len %u\n", this, index, len);

    DEBUG_ASSERT(len > 0 && ispow2(len) && len < UINT16_MAX);
    DEBUG_ASSERT(index < MAX_VIRTIO_RINGS);

    if (len == 0 || !ispow2(len) || len >= UINT16_MAX)
        return ERR_INVALID_ARGS;

    vring *ring = &ring_[index];

    /* allocate a ring */
    size_t size = vring_size(len, PAGE_SIZE);
    LTRACEF("need %zu bytes\n", size);

#if WITH_KERNEL_VM
    void *vptr;
    status_t err = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "virtio_ring", size, &vptr, 0, 0, ARCH_MMU_FLAG_UNCACHED_DEVICE);
    if (err < 0)
        return ERR_NO_MEMORY;

    LTRACEF("allocated virtio_ring at va %p\n", vptr);

    /* compute the physical address */
    paddr_t pa;
    pa = vaddr_to_paddr(vptr);
    if (pa == 0) {
        return ERR_NO_MEMORY;
    }

    LTRACEF("virtio_ring at pa 0x%lx\n", pa);
#else
    void *vptr = memalign(PAGE_SIZE, size);
    if (!vptr)
        return ERR_NO_MEMORY;

    LTRACEF("ptr %p\n", vptr);
    memset(vptr, 0, size);

    /* compute the physical address */
    paddr_t pa = (paddr_t)vptr;
#endif

    /* initialize the ring */
    vring_init(ring, len, vptr, PAGE_SIZE);
    ring_[index].free_list = 0xffff;
    ring_[index].free_count = 0;
    ring_len_[index] = len;

    /* add all the descriptors to the free list */
    for (uint i = 0; i < len; i++) {
        virtio_free_desc(index, i);
    }

    /* register the ring with the device */
    DEBUG_ASSERT(mmio_config_);
    mmio_config_->guest_page_size = PAGE_SIZE;
    mmio_config_->queue_sel = index;
    mmio_config_->queue_num = len;
    mmio_config_->queue_align = PAGE_SIZE;
    mmio_config_->queue_pfn = pa / PAGE_SIZE;

    /* mark the ring active */
    active_rings_bitmap_ |= (1 << index);

    return NO_ERROR;
}

enum handler_return virtio_device::virtio_mmio_irq(void *arg) {
    virtio_device *dev = (virtio_device *)arg;
    LTRACEF("dev %p, index %u\n", dev, dev->index_);

    uint32_t irq_status = dev->mmio_config_->interrupt_status;
    LTRACEF("status 0x%x\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
        // XXX is this safe?
        dev->mmio_config_->interrupt_ack = 0x1;

        /* cycle through all the active rings */
        for (uint r = 0; r < MAX_VIRTIO_RINGS; r++) {
            if ((dev->active_rings_bitmap_ & (1<<r)) == 0)
                continue;

            vring *ring = &dev->ring_[r];
            LTRACEF("ring %u: used flags 0x%hx idx 0x%hx last_used %u\n", r, ring->used->flags, ring->used->idx, ring->last_used);

            uint cur_idx = ring->used->idx;
            for (uint i = ring->last_used; i != (cur_idx & ring->num_mask); i = (i + 1) & ring->num_mask) {
                LTRACEF("looking at idx %u\n", i);

                // process chain
                vring_used_elem *used_elem = &ring->used->ring[i];
                LTRACEF("id %u, len %u\n", used_elem->id, used_elem->len);

                DEBUG_ASSERT(dev->irq_driver_callback_);
                if (dev->irq_driver_callback_(dev, r, used_elem) == INT_RESCHEDULE) {
                    ret = INT_RESCHEDULE;
                }

                ring->last_used = (ring->last_used + 1) & ring->num_mask;
            }
        }
    }
    if (irq_status & 0x2) { /* config change */
        dev->mmio_config_->interrupt_ack = 0x2;

        if (dev->config_change_callback_) {
            if (dev->config_change_callback_(dev) == INT_RESCHEDULE) {
                ret = INT_RESCHEDULE;
            }
        }
    }

    LTRACEF("exiting irq\n");

    return ret;
}

