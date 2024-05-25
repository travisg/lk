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

struct virtio_mmio_config;

#define MAX_VIRTIO_RINGS 4

class virtio_device {
public:
    virtio_device() = default;
    virtual ~virtio_device() = default;

    void virtio_reset_device();
    void virtio_status_acknowledge_driver();
    uint32_t virtio_read_host_feature_word(uint32_t word);
    void virtio_set_guest_features(uint32_t word, uint32_t features);
    void virtio_status_driver_ok();

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

    void virtio_kick(uint ring_idnex);

    static enum handler_return virtio_mmio_irq(void *arg);

//private:
    bool valid_ = {};

    uint index_ = {};
    uint irq_ = {};

    volatile struct virtio_mmio_config *mmio_config_ = {};
    void *config_ptr_ = {};

    void *priv_ = {}; /* a place for the driver to put private data */

    enum handler_return (*irq_driver_callback_)(virtio_device *dev, uint ring, const vring_used_elem *e) = {};
    enum handler_return (*config_change_callback_)(virtio_device *dev) = {};

    /* virtio rings */
    uint32_t active_rings_bitmap_ = {};
    uint16_t ring_len_[MAX_VIRTIO_RINGS] = {};
    vring ring_[MAX_VIRTIO_RINGS];
};


void dump_mmio_config(const volatile virtio_mmio_config *mmio);
void virtio_dump_desc(const vring_desc *desc);

