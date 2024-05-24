/*
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lk/compiler.h>
#include <assert.h>
#include <lk/list.h>
#include <sys/types.h>
#include <dev/virtio/virtio_ring.h>

__BEGIN_CDECLS

/* detect a virtio mmio hardware block
 * returns number of devices found */
int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride);

#define MAX_VIRTIO_RINGS 4

struct virtio_mmio_config;

struct virtio_device {
    bool valid;

    uint index;
    uint irq;

    volatile struct virtio_mmio_config *mmio_config;
    void *config_ptr;

    void *priv; /* a place for the driver to put private data */

    enum handler_return (*irq_driver_callback)(struct virtio_device *dev, uint ring, const struct vring_used_elem *e);
    enum handler_return (*config_change_callback)(struct virtio_device *dev);

    /* virtio rings */
    uint32_t active_rings_bitmap;
    struct vring ring[MAX_VIRTIO_RINGS];
};

void virtio_reset_device(struct virtio_device *dev);
void virtio_status_acknowledge_driver(struct virtio_device *dev);
uint32_t virtio_read_host_feature_word(struct virtio_device *dev, uint32_t word);
void virtio_set_guest_features(struct virtio_device *dev, uint32_t word, uint32_t features);
void virtio_status_driver_ok(struct virtio_device *dev);

/* api used by devices to interact with the virtio bus */
status_t virtio_alloc_ring(struct virtio_device *dev, uint index, uint16_t len) __NONNULL();

/* add a descriptor at index desc_index to the free list on ring_index */
void virtio_free_desc(struct virtio_device *dev, uint ring_index, uint16_t desc_index);

/* allocate a descriptor off the free list, 0xffff is error */
uint16_t virtio_alloc_desc(struct virtio_device *dev, uint ring_index);

/* allocate a descriptor chain the free list */
struct vring_desc *virtio_alloc_desc_chain(struct virtio_device *dev, uint ring_index, size_t count, uint16_t *start_index);

static inline struct vring_desc *virtio_desc_index_to_desc(struct virtio_device *dev, uint ring_index, uint16_t desc_index) {
    DEBUG_ASSERT(desc_index != 0xffff);
    return &dev->ring[ring_index].desc[desc_index];
}

void virtio_dump_desc(const struct vring_desc *desc);

/* submit a chain to the avail list */
void virtio_submit_chain(struct virtio_device *dev, uint ring_index, uint16_t desc_index);

void virtio_kick(struct virtio_device *dev, uint ring_idnex);

__END_CDECLS

