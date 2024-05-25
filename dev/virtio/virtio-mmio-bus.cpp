/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio/virtio-mmio-bus.h>

#include <assert.h>
#include <stdint.h>
#include <lk/trace.h>
#include <lk/err.h>
#include <lk/pow2.h>
#include <arch/ops.h>
#include <dev/virtio.h>
#include <dev/virtio/virtio-device.h>
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

// V1 config
struct virtio_mmio_config {
    /* 0x00 */
    uint32_t magic;
    uint32_t version;
    uint32_t device_id;
    uint32_t vendor_id;
    /* 0x10 */
    uint32_t host_features;
    uint32_t host_features_sel;
    uint32_t _reserved0[2];
    /* 0x20 */
    uint32_t guest_features;
    uint32_t guest_features_sel;
    uint32_t guest_page_size;
    uint32_t _reserved1[1];
    /* 0x30 */
    uint32_t queue_sel;
    uint32_t queue_num_max;
    uint32_t queue_num;
    uint32_t queue_align;
    /* 0x40 */
    uint32_t queue_pfn;
    uint32_t _reserved2[3];
    /* 0x50 */
    uint32_t queue_notify;
    uint32_t _reserved3[3];
    /* 0x60 */
    uint32_t interrupt_status;
    uint32_t interrupt_ack;
    uint32_t _reserved4[2];
    /* 0x70 */
    uint32_t status;
    uint8_t  _reserved5[0x8c];
    /* 0x100 */
    uint32_t config[0];
};

STATIC_ASSERT(sizeof(struct virtio_mmio_config) == 0x100);

#define VIRTIO_MMIO_MAGIC 0x74726976 // 'virt'

#define VIRTIO_STATUS_ACKNOWLEDGE (1<<0)
#define VIRTIO_STATUS_DRIVER      (1<<1)
#define VIRTIO_STATUS_DRIVER_OK   (1<<2)
#define VIRTIO_STATUS_FEATURES_OK (1<<3)
#define VIRTIO_STATUS_DEVICE_NEEDS_RESET (1<<6)
#define VIRTIO_STATUS_FAILED      (1<<7)

// TODO: switch to using reg.h mmio_ accessors
void virtio_mmio_bus::virtio_reset_device() {
    mmio_config_->status = 0;
}

void virtio_mmio_bus::virtio_status_acknowledge_driver() {
    mmio_config_->status |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
}

void virtio_mmio_bus::virtio_status_driver_ok() {
    mmio_config_->status |= VIRTIO_STATUS_DRIVER_OK;
}

void virtio_mmio_bus::virtio_set_guest_features(uint32_t word, uint32_t features) {
    mmio_config_->guest_features_sel = word;
    mmio_config_->guest_features = features;
}

uint32_t virtio_mmio_bus::virtio_read_host_feature_word(uint32_t word) {
    mmio_config_->host_features_sel = word;
    return mmio_config_->host_features;
}

void virtio_mmio_bus::virtio_kick(uint16_t ring_index) {
    LTRACEF("bus %p, ring %u\n", this, ring_index);

    DEBUG_ASSERT(ring_index < virtio_device::MAX_VIRTIO_RINGS);

    mmio_config_->queue_notify = ring_index;
    mb();
}

void virtio_mmio_bus::register_ring(uint32_t page_size, uint32_t queue_sel, uint32_t queue_num, uint32_t queue_align, uint32_t queue_pfn) {
    DEBUG_ASSERT(mmio_config_);
    mmio_config_->guest_page_size = page_size;
    mmio_config_->queue_sel = queue_sel;
    mmio_config_->queue_num = queue_num;
    mmio_config_->queue_align = queue_align;
    mmio_config_->queue_pfn = queue_pfn;
}

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

enum handler_return virtio_mmio_bus::virtio_mmio_irq(void *arg) {
    auto *dev = static_cast<virtio_device *>(arg);
    virtio_mmio_bus *bus = reinterpret_cast<virtio_mmio_bus *>(dev->bus());

    LTRACEF("dev %p, bus %p\n", dev, bus);

    uint32_t irq_status = bus->mmio_config_->interrupt_status;
    LTRACEF("status %#x\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
        // XXX is this safe?
        bus->mmio_config_->interrupt_ack = 0x1;

        auto _ret = dev->handle_queue_interrupt();
        if (_ret == INT_RESCHEDULE) {
            ret = _ret;
        }

    }
    if (irq_status & 0x2) { /* config change */
        bus->mmio_config_->interrupt_ack = 0x2;

        auto _ret = dev->handle_config_interrupt();
        if (_ret == INT_RESCHEDULE) {
            ret = _ret;
        }
    }

    LTRACEF("exiting irq\n");

    return ret;
}

int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride) {
    LTRACEF("ptr %p, count %u\n", ptr, count);

    DEBUG_ASSERT(ptr);
    DEBUG_ASSERT(irqs);

    /* allocate an array big enough to hold a list of pointers to devices */
    static virtio_device **devices = new virtio_device *[count] {};
    if (!devices)
        return ERR_NO_MEMORY;

    int found = 0;
    for (uint i = 0; i < count; i++) {
        volatile auto *mmio = reinterpret_cast<virtio_mmio_config *>(static_cast<uint8_t *>(ptr) + i * stride);

        auto *bus = new virtio_mmio_bus(mmio);
        auto *dev = new virtio_device(bus);
        devices[i] = dev;

        dev->index_ = i;
        dev->irq_ = irqs[i];
        dev->config_ptr_ = (void *)mmio->config;

        mask_interrupt(irqs[i]);
        register_int_handler(irqs[i], &virtio_mmio_bus::virtio_mmio_irq, static_cast<void *>(dev));

        LTRACEF("looking at %p: magic 0x%x version 0x%x did 0x%x vid 0x%x\n",
                mmio, mmio->magic, mmio->version, mmio->device_id, mmio->vendor_id);

        if (mmio->magic != VIRTIO_MMIO_MAGIC) {
            continue;
        }

        // TODO: handle version 2

        if (LOCAL_TRACE) {
            if (mmio->device_id != 0) {
                dump_mmio_config(mmio);
            }
        }

#if WITH_DEV_VIRTIO_BLOCK
        if (mmio->device_id == 2) { // block device
            LTRACEF("found block device\n");

            status_t err = virtio_block_init(dev, bus->virtio_read_host_feature_word(0));
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

            status_t err = virtio_9p_init(dev, bus->virtio_read_host_feature_word((0)));
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

            status_t err = virtio_gpu_init(dev, bus->virtio_read_host_feature_word( 0));
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

