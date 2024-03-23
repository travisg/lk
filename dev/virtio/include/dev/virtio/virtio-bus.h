/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <stdint.h>
#include <platform/interrupts.h>

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

    uint64_t virtio_read_host_feature_word_64(uint32_t word) {
        return virtio_read_host_feature_word(word) | static_cast<uint64_t>(virtio_read_host_feature_word(word + 1)) << 32;
    }

    // A simple set of routines to handle a single IRQ.
    // TODO: rethink where this goes in a more complicated MSI based solution.
    void set_irq(uint32_t irq) { irq_ = irq; }

    void mask_interrupt() {
        ::mask_interrupt(irq_);
    }

    void unmask_interrupt() {
        ::unmask_interrupt(irq_);
    }

private:
    uint32_t irq_ {};
};