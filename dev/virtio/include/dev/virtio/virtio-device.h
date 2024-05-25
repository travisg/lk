/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lk/compiler.h>
#include <assert.h>
#include <sys/types.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtio-bus.h>

struct virtio_mmio_config;

// TODO: move as const inside virio_device
#define MAX_VIRTIO_RINGS 4

class virtio_device {
public:
    virtio_device() = default;
    virtual ~virtio_device() = default;

    /* api used by devices to interact with the virtio bus */
    status_t virtio_alloc_ring(uint index, uint16_t len) __NONNULL();

    /* add a descriptor at index desc_index to the free list on ring_index */
    void virtio_free_desc(uint ring_index, uint16_t desc_index);

    /* allocate a descriptor off the free list, 0xffff is error */
    uint16_t virtio_alloc_desc(uint ring_index);

    /* allocate a descriptor chain the free list */
    vring_desc *virtio_alloc_desc_chain(uint ring_index, size_t count, uint16_t *start_index);

    inline vring_desc *virtio_desc_index_to_desc(uint ring_index, uint16_t desc_index) {
        DEBUG_ASSERT(desc_index != 0xffff);
        return &ring_[ring_index].desc[desc_index];
    }

    /* submit a chain to the avail list */
    void virtio_submit_chain(uint ring_index, uint16_t desc_index);

    static enum handler_return virtio_mmio_irq(void *arg);

    // accessors
    void *priv() { return priv_; }
    const void *priv() const { return priv_; }

    void set_priv(void *p) { priv_ = p; }

    virtio_bus *bus() { return bus_; }

    void *get_config_ptr() { return config_ptr_; }
    const void *get_config_ptr() const { return config_ptr_; }

    using irq_driver_callback = enum handler_return (*)(virtio_device *dev, uint ring, const vring_used_elem *e);
    using config_change_callback = enum handler_return (*)(virtio_device *dev);

    void set_irq_callbacks(irq_driver_callback irq, config_change_callback config) {
        irq_driver_callback_ = irq;
        config_change_callback_ = config;
    }

private:
    // XXX move this into constructor
    friend int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride);

    // mmio or pci
    virtio_bus *bus_ = {};

    // points into bus's configuration spot
    // TODO: is this feasible for both PCI and mmio?
    void *config_ptr_ = {};

    void *priv_ = {}; /* a place for the driver to put private data */

    bool valid_ = {};

    uint index_ = {};
    uint irq_ = {};

    irq_driver_callback irq_driver_callback_ = {};
    config_change_callback config_change_callback_ = {};

    /* virtio rings */
    uint32_t active_rings_bitmap_ = {};
    uint16_t ring_len_[MAX_VIRTIO_RINGS] = {};
    vring ring_[MAX_VIRTIO_RINGS];
};


void dump_mmio_config(const volatile virtio_mmio_config *mmio);
void virtio_dump_desc(const vring_desc *desc);

