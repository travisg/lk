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

#include <dev/bus/pci.h>

struct virtio_pci_common_cfg;
class virtio_device;

class virtio_pci_bus final : public virtio_bus {
public:
    virtio_pci_bus() = default;
    ~virtio_pci_bus() override = default;

    status_t init(virtio_device *dev, pci_location_t loc, size_t index);

    void virtio_reset_device() override;
    void virtio_status_acknowledge_driver() override;
    uint32_t virtio_read_host_feature_word(uint32_t word) override;
    void virtio_set_guest_features(uint32_t word, uint32_t features) override;
    void virtio_status_driver_ok() override;
    void virtio_kick(uint16_t ring_index) override;
    void register_ring(uint32_t page_size, uint32_t queue_sel, uint32_t queue_num, uint32_t queue_align, uint32_t queue_pfn) override;

    volatile virtio_pci_common_cfg *common_config() {
        return  reinterpret_cast<volatile virtio_pci_common_cfg *>(config_ptr(common_cfg_));
    }

    void *device_config() {
        return  reinterpret_cast<void *>(config_ptr(device_cfg_));
    }

private:
    static handler_return virtio_pci_irq(void *arg);

    struct config_pointer {
        bool valid;
        int bar;
        size_t offset;
        size_t length;
    };

    struct mapped_bars {
        bool mapped;
        uint8_t *vaddr;
    };

    virtio_device *dev_ = {};

    pci_location_t loc_ = {};

    mapped_bars bar_map_[6] = {};

    config_pointer common_cfg_ = {};
    config_pointer notify_cfg_ = {};
    config_pointer isr_cfg_ = {};
    config_pointer device_cfg_ = {};
    config_pointer pci_cfg_ = {};

    uint32_t notify_offset_multiplier_ = {};

    // Given one of the config_pointer structs, return a uint8_t * pointer
    // to its mapping.
    uint8_t *config_ptr(const config_pointer &cfg) {
        if (!cfg.valid) {
            return nullptr;
        }

        auto &bar = bar_map_[cfg.bar];
        return bar.vaddr + cfg.offset;
    }
};