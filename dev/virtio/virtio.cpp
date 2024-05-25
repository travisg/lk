/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio.h>
#include <dev/virtio/virtio_ring.h>

#include <lk/debug.h>
#include <assert.h>
#include <lk/trace.h>
#include <lk/compiler.h>
#include <lk/list.h>
#include <lk/err.h>
#include <stdlib.h>
#include <string.h>
#include <lk/pow2.h>
#include <lk/init.h>
#include <kernel/thread.h>
#include <platform/interrupts.h>
#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#include "virtio_priv.h"

#if WITH_DEV_VIRTIO_BLOCK
#include <dev/virtio/block.h>
#endif
#if WITH_DEV_VIRTIO_NET
#include <dev/virtio/net.h>
#endif
#if WITH_DEV_VIRTIO_GPU
#include <dev/virtio/gpu.h>
#endif
#if WITH_DEV_VIRTIO_9P
#include <dev/virtio/9p.h>
#endif

#define LOCAL_TRACE 0

static virtio_device *devices;

void dump_mmio_config(const volatile virtio_mmio_config *mmio) {
    printf("mmio at %p\n", mmio);
    printf("\tmagic 0x%x\n", mmio->magic);
    printf("\tversion 0x%x\n", mmio->version);
    printf("\tdevice_id 0x%x\n", mmio->device_id);
    printf("\tvendor_id 0x%x\n", mmio->vendor_id);
    printf("\thost_features 0x%x\n", mmio->host_features);
    printf("\tguest_features 0x%x\n", mmio->guest_features);
    printf("\tguest_features_sel 0x%x\n", mmio->guest_features_sel);
    printf("\tguest_page_size %u\n", mmio->guest_page_size);
    printf("\tqnum %u\n", mmio->queue_num);
    printf("\tqnum_max %u\n", mmio->queue_num_max);
    printf("\tqnum_align %u\n", mmio->queue_align);
    printf("\tqnum_pfn %u\n", mmio->queue_pfn);
    printf("\tstatus 0x%x\n", mmio->status);
}

void virtio_dump_desc(const vring_desc *desc) {
    printf("vring descriptor %p\n", desc);
    printf("\taddr  0x%llx\n", desc->addr);
    printf("\tlen   0x%x\n", desc->len);
    printf("\tflags 0x%hx\n", desc->flags);
    printf("\tnext  0x%hx\n", desc->next);
}

int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride) {
    LTRACEF("ptr %p, count %u\n", ptr, count);

    DEBUG_ASSERT(ptr);
    DEBUG_ASSERT(irqs);
    DEBUG_ASSERT(!devices);

    /* allocate an array big enough to hold a list of devices */
    devices = new virtio_device[count];
    if (!devices)
        return ERR_NO_MEMORY;

    int found = 0;
    for (uint i = 0; i < count; i++) {
        volatile auto *mmio = (virtio_mmio_config *)((uint8_t *)ptr + i * stride);
        virtio_device *dev = &devices[i];

        dev->index_ = i;
        dev->irq_ = irqs[i];

        mask_interrupt(irqs[i]);
        register_int_handler(irqs[i], &virtio_device::virtio_mmio_irq, (void *)dev);

        LTRACEF("looking at %p: magic 0x%x version 0x%x did 0x%x vid 0x%x\n",
                mmio, mmio->magic, mmio->version, mmio->device_id, mmio->vendor_id);

        if (mmio->magic != VIRTIO_MMIO_MAGIC) {
            continue;
        }

        // TODO: handle version 2

#if LOCAL_TRACE
        if (mmio->device_id != 0) {
            dump_mmio_config(mmio);
        }
#endif

#if WITH_DEV_VIRTIO_BLOCK
        if (mmio->device_id == 2) { // block device
            LTRACEF("found block device\n");

            dev->mmio_config_ = mmio;
            dev->config_ptr_ = (void *)mmio->config;

            status_t err = virtio_block_init(dev, dev->virtio_read_host_feature_word(0));
            if (err >= 0) {
                // good device
                dev->valid_ = true;

                if (dev->irq_driver_callback_)
                    unmask_interrupt(dev->irq_);
            }
        }
#endif // WITH_DEV_VIRTIO_BLOCK
#if WITH_DEV_VIRTIO_NET
        if (mmio->device_id == 1) { // network device
            LTRACEF("found net device\n");

            dev->mmio_config_ = mmio;
            dev->config_ptr_ = (void *)mmio->config;

            status_t err = virtio_net_init(dev);
            if (err >= 0) {
                // good device
                dev->valid_ = true;

                if (dev->irq_driver_callback_)
                    unmask_interrupt(dev->irq_);
            }
        }
#endif // WITH_DEV_VIRTIO_NET
#if WITH_DEV_VIRTIO_9P
        if (mmio->device_id == 9) { // 9p device
            LTRACEF("found 9p device\n");

            dev->mmio_config_ = mmio;
            dev->config_ptr_ = (void *)mmio->config;

            status_t err = virtio_9p_init(dev, mmio->host_features);
            if (err >= 0) {
                // good device
                dev->valid_ = true;

                if (dev->irq_driver_callback_)
                    unmask_interrupt(dev->irq_);

                virtio_9p_start(dev);
            }
        }
#endif // WITH_DEV_VIRTIO_9P
#if WITH_DEV_VIRTIO_GPU
        if (mmio->device_id == 0x10) { // virtio-gpu
            LTRACEF("found gpu device\n");

            dev->mmio_config_ = mmio;
            dev->config_ptr_ = (void *)mmio->config;

            status_t err = virtio_gpu_init(dev, dev->virtio_read_host_feature_word(0));
            if (err >= 0) {
                // good device
                dev->valid_ = true;

                if (dev->irq_driver_callback_)
                    unmask_interrupt(dev->irq_);

                virtio_gpu_start(dev);
            }
        }
#endif // WITH_DEV_VIRTIO_GPU

        if (dev->valid_)
            found++;
    }

    return found;
}

static void virtio_init(uint level) {
}

LK_INIT_HOOK(virtio, &virtio_init, LK_INIT_LEVEL_THREADING);

