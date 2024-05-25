/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <stdint.h>

class virtio_bus {
public:
    virtio_bus() = default;
    virtual ~virtio_bus() = default;

    virtual void virtio_reset_device() = 0;
    virtual void virtio_status_acknowledge_driver() = 0;
    virtual uint32_t virtio_read_host_feature_word(uint32_t word) = 0;
    virtual void virtio_set_guest_features(uint32_t word, uint32_t features) = 0;
    virtual void virtio_status_driver_ok() = 0;
    virtual void virtio_kick(uint16_t ring_index) = 0;
    virtual void register_ring(uint32_t page_size, uint32_t queue_sel, uint32_t queue_num, uint32_t queue_align, uint32_t queue_pfn) = 0;
};