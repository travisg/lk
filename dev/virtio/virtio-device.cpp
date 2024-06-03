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
#include <sys/types.h>
#include <dev/virtio/virtio-mmio-bus.h>

#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#include "virtio_priv.h"

#define LOCAL_TRACE 1

void virtio_device::virtio_free_desc(uint ring_index, uint16_t desc_index) {
    LTRACEF("dev %p ring %u index %u free_count %u\n", this, ring_index, desc_index, ring_[ring_index].free_count);

    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);
    DEBUG_ASSERT(desc_index < ring_len_[ring_index]);

    vring &ring = ring_[ring_index];

    ring.desc[desc_index].next = ring.free_list;
    ring.free_list = desc_index;
    ring.free_count++;
}

uint16_t virtio_device::virtio_alloc_desc(uint ring_index) {
    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

    vring &ring = ring_[ring_index];

    if (ring.free_count == 0)
        return 0xffff;

    DEBUG_ASSERT(ring.free_list != 0xffff);

    uint16_t i = ring.free_list;
    vring_desc *desc = &ring.desc[i];
    ring.free_list = desc->next;

    ring.free_count--;

    return i;
}

vring_desc *virtio_device::virtio_alloc_desc_chain(uint ring_index, size_t count, uint16_t *start_index) {
    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

    vring &ring = ring_[ring_index];

    if (ring.free_count < count)
        return nullptr;

    /* start popping entries off the chain */
    vring_desc *last = nullptr;
    uint16_t last_index = 0;
    while (count > 0) {
        uint16_t i = ring.free_list;
        vring_desc *desc = &ring.desc[i];

        ring.free_list = desc->next;
        ring.free_count--;

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

    vring &ring = ring_[ring_index];

    /* add the chain to the available list */
    vring_avail *avail = ring.avail;

    avail->ring[avail->idx & ring.num_mask] = desc_index;
    mb();
    avail->idx++;

#if LOCAL_TRACE
    hexdump(avail, 16);
#endif
}

status_t virtio_device::virtio_alloc_ring(uint index, uint16_t len) {
    LTRACEF("dev %p, index %u, len %u\n", this, index, len);

    DEBUG_ASSERT(len > 0 && ispow2(len) && len < UINT16_MAX);
    DEBUG_ASSERT(index < MAX_VIRTIO_RINGS);

    if (len == 0 || !ispow2(len) || len >= UINT16_MAX)
        return ERR_INVALID_ARGS;
    if (index >= MAX_VIRTIO_RINGS)
        return ERR_INVALID_ARGS;

    vring &ring = ring_[index];

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
    vring_init(&ring, len, vptr, PAGE_SIZE);
    ring.free_list = 0xffff;
    ring.free_count = 0;
    ring_len_[index] = len;

    /* add all the descriptors to the free list */
    for (uint i = 0; i < len; i++) {
        virtio_free_desc(index, i);
    }

    /* register the ring with the device */
    bus()->register_ring(PAGE_SIZE, index, len, PAGE_SIZE, pa / PAGE_SIZE);

    /* mark the ring active */
    active_rings_bitmap_ |= (1 << index);

    return NO_ERROR;
}

handler_return virtio_device::handle_queue_interrupt() {
    LTRACE_ENTRY;
    handler_return ret = INT_NO_RESCHEDULE;

    /* cycle through all the active rings */
    for (uint r = 0; r < MAX_VIRTIO_RINGS; r++) {
        if ((active_rings_bitmap_ & (1<<r)) == 0)
            continue;

        vring &ring = ring_[r];

        LTRACEF("desc %p, avail %p, used %p\n", ring.desc, ring.avail, ring.used);
        LTRACEF("ring %u: used flags 0x%hx idx 0x%hx last_used %u\n", r, ring.used->flags, ring.used->idx, ring.last_used);

        uint cur_idx = ring.used->idx;
        for (uint i = ring.last_used; i != (cur_idx & ring.num_mask); i = (i + 1) & ring.num_mask) {
            LTRACEF("looking at idx %u\n", i);

            // process chain
            vring_used_elem *used_elem = &ring.used->ring[i];
            LTRACEF("id %u, len %u\n", used_elem->id, used_elem->len);

            DEBUG_ASSERT(irq_driver_callback_);
            if (irq_driver_callback_(this, r, used_elem) == INT_RESCHEDULE) {
                ret = INT_RESCHEDULE;
            }

            ring.last_used = (ring.last_used + 1) & ring.num_mask;
        }
    }

    return ret;
}

handler_return virtio_device::handle_config_interrupt() {
    LTRACE_ENTRY;
    handler_return ret = INT_NO_RESCHEDULE;
    if (config_change_callback_) {
        if (config_change_callback_(this) == INT_RESCHEDULE) {
            ret = INT_RESCHEDULE;
        }
    }

    return ret;
}