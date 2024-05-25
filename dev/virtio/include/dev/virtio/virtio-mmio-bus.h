/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <stdint.h>
#include <sys/types.h>

#include <dev/virtio/virtio-bus.h>

struct virtio_mmio_config;

class virtio_mmio_bus final : public virtio_bus {
public:
    explicit virtio_mmio_bus(volatile virtio_mmio_config *config) : mmio_config_(config) {}
    ~virtio_mmio_bus() override = default;

    void virtio_reset_device() override;
    void virtio_status_acknowledge_driver() override;
    uint32_t virtio_read_host_feature_word(uint32_t word) override;
    void virtio_set_guest_features(uint32_t word, uint32_t features) override;
    void virtio_status_driver_ok() override;
    void virtio_kick(uint16_t ring_index) override;

    void register_ring(uint32_t page_size, uint32_t queue_sel, uint32_t queue_num, uint32_t queue_align, uint32_t queue_pfn) override;

    static handler_return virtio_mmio_irq(void *arg);

private:
    volatile struct virtio_mmio_config *mmio_config_;
};

void dump_mmio_config(const volatile virtio_mmio_config *mmio);