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
#include <arch/ops.h>
#include <dev/virtio/virtio-device.h>

#include "virtio_priv.h"

#define LOCAL_TRACE 0

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
    LTRACEF("dev %p, ring %u\n", this, ring_index);

    DEBUG_ASSERT(ring_index < MAX_VIRTIO_RINGS);

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

