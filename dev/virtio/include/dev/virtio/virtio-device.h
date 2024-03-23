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
#include <dev/virtio.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtio-bus.h>
#include <platform/interrupts.h>

class virtio_device {
public:
    explicit virtio_device(virtio_bus *bus) : bus_(bus) {}
    virtual ~virtio_device() {
        delete bus_;
    }

    /* api used by devices to interact with the virtio bus */
    status_t virtio_alloc_ring(uint index, uint16_t len);

    /* add a descriptor at index desc_index to the free list on ring_index */
    void virtio_free_desc(uint ring_index, uint16_t desc_index);

    /* allocate a descriptor off the free list, 0xffff is error */
    uint16_t virtio_alloc_desc(uint ring_index);

    /* allocate a descriptor chain the free list */
    vring_desc *virtio_alloc_desc_chain(uint ring_index, size_t count, uint16_t *start_index);

    inline vring_desc *virtio_desc_index_to_desc(uint ring_index, uint16_t desc_index) {
        DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);
        DEBUG_ASSERT(desc_index != 0xffff);
        return &ring_[ring_index].desc[desc_index];
    }

    /* submit a chain to the avail list */
    void virtio_submit_chain(uint ring_index, uint16_t desc_index);

    // accessors
    void *priv() { return priv_; }
    const void *priv() const { return priv_; }

    void set_priv(void *p) { priv_ = p; }

    virtio_bus *bus() { return bus_; }

    void *get_config_ptr() { return config_ptr_; }
    const void *get_config_ptr() const { return config_ptr_; }
    void set_config_ptr(void *ptr) { config_ptr_ = ptr; }

    using irq_driver_callback = enum handler_return (*)(virtio_device *dev, uint ring, const vring_used_elem *e);
    using config_change_callback = enum handler_return (*)(virtio_device *dev);

    void set_irq_callbacks(irq_driver_callback irq, config_change_callback config) {
        irq_driver_callback_ = irq;
        config_change_callback_ = config;
    }

    // Interrupt handler callbacks from the bus layer, which is responsible
    // for the first layer of IRQ handling
    handler_return handle_queue_interrupt();
    handler_return handle_config_interrupt();

    // TODO: allow an aribitrary number of rings
    static const size_t MAX_VIRTIO_RINGS = 4;

private:
    // mmio or pci
    virtio_bus *bus_ = {};

    // points into bus's configuration spot
    void *config_ptr_ = {};

    // a place for the driver to put private data, usually a pointer to device
    // specific details.
    void *priv_ = {};

    // interrupt handlers that the device-specific layer registers with our layer
    irq_driver_callback irq_driver_callback_ = {};
    config_change_callback config_change_callback_ = {};

    /* virtio rings */
    uint32_t active_rings_bitmap_ = {};
    uint16_t ring_len_[MAX_VIRTIO_RINGS] = {};
    vring ring_[MAX_VIRTIO_RINGS] = {};
};


