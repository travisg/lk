/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include "lk/debug.h"
#if WITH_DEV_BUS_PCI

#include <dev/virtio/virtio-pci-bus.h>

#include <stdlib.h>
#include <dev/virtio.h>
#include <dev/virtio/virtio-device.h>
#include <arch/ops.h>
#include <inttypes.h>
#include <platform/interrupts.h>
#include <lk/init.h>
#include <lk/err.h>
#include <lk/trace.h>
#include <lk/list.h>
#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif
#if WITH_DEV_VIRTIO_BLOCK
#include <dev/virtio/block.h>
#endif

#define LOCAL_TRACE 0

#include <dev/bus/pci.h>

#include "virtio_priv.h"

struct virtio_pci_devices {
    uint16_t device_id;
    bool legacy;
    status_t (*init)(pci_location_t loc, const virtio_pci_devices &dev_table_entry, size_t index);
};

static status_t init_block(pci_location_t loc, const virtio_pci_devices &dev_table_entry, size_t index);

const virtio_pci_devices devices[] = {
    { 0x1000, true, nullptr }, // transitional network
    { 0x1001, true, &init_block }, // transitional block
    { 0x1009, true, nullptr }, // legacy virtio 9p
    { 0x1041, false, nullptr }, // non-transitional network
    { 0x1042, false, &init_block }, // non-transitional block
    { 0x1043, false, nullptr }, // non-transitional console
    { 0x1050, false, nullptr }, // non-transitional gpu
    { 0x1052, false, nullptr }, // non-transitional input
};

struct virtio_pci_cap {
    uint8_t cap_vndr;
    uint8_t cap_next;
    uint8_t cap_len;
    /* Common configuration */
#define VIRTIO_PCI_CAP_COMMON_CFG 1
    /* Notifications */
#define VIRTIO_PCI_CAP_NOTIFY_CFG 2
    /* ISR Status */
#define VIRTIO_PCI_CAP_ISR_CFG 3
    /* Device specific configuration */
#define VIRTIO_PCI_CAP_DEVICE_CFG 4
    /* PCI configuration access */
#define VIRTIO_PCI_CAP_PCI_CFG 5
    /* Shared memory region */
#define VIRTIO_PCI_CAP_SHARED_MEMORY_CFG 8
    /* Vendor-specific data */
#define VIRTIO_PCI_CAP_VENDOR_CFG 9
    uint8_t cfg_type;
    uint8_t bar;
    uint8_t id;
    uint8_t padding[2];
    uint32_t offset;
    uint32_t length;
};

STATIC_ASSERT(sizeof(virtio_pci_cap) == 16);

static void dump_pci_cap(const virtio_pci_cap *cap) {
    const char *type;

    switch (cap->cfg_type) {
        case VIRTIO_PCI_CAP_COMMON_CFG: type = "common"; break;
        case VIRTIO_PCI_CAP_NOTIFY_CFG: type = "notify"; break;
        case VIRTIO_PCI_CAP_ISR_CFG: type = "isr"; break;
        case VIRTIO_PCI_CAP_DEVICE_CFG: type = "device"; break;
        case VIRTIO_PCI_CAP_PCI_CFG: type = "pci"; break;
        case VIRTIO_PCI_CAP_SHARED_MEMORY_CFG: type = "shared mem"; break;
        case VIRTIO_PCI_CAP_VENDOR_CFG: type = "vendor"; break;
        default: type = "unknown"; break;
    }

    printf("PCI capability: vendor %#hhx next %#hhx len %#hhx type %#hhx (%s) bar %#hhx id %#hhx offset %#x length %#x\n",
            cap->cap_vndr, cap->cap_next, cap->cap_len, cap->cfg_type, type, cap->bar, cap->id, cap->offset, cap->length);
}

struct virtio_pci_notify_cap {
    virtio_pci_cap cap;
    uint32_t notify_off_multiplier;
};

STATIC_ASSERT(sizeof(virtio_pci_notify_cap) == 20);

struct virtio_pci_common_cfg {
    /* About the whole device. */
    uint32_t device_feature_select; /* read-write */
    uint32_t device_feature; /* read-only for driver */
    uint32_t driver_feature_select; /* read-write */
    uint32_t driver_feature; /* read-write */
    uint16_t config_msix_vector; /* read-write */
    uint16_t num_queues; /* read-only for driver */
    uint8_t device_status; /* read-write */
    uint8_t config_generation; /* read-only for driver */

    /* About a specific virtqueue. */
    uint16_t queue_select; /* read-write */
    uint16_t queue_size; /* read-write */
    uint16_t queue_msix_vector; /* read-write */
    uint16_t queue_enable; /* read-write */
    uint16_t queue_notify_off; /* read-only for driver */
    uint64_t queue_desc; /* read-write */
    uint64_t queue_driver; /* read-write */
    uint64_t queue_device; /* read-write */
    uint16_t queue_notif_config_data; /* read-only for driver */
    uint16_t queue_reset; /* read-write */

    /* About the administration virtqueue. */
    uint16_t admin_queue_index; /* read-only for driver */
    uint16_t admin_queue_num; /* read-only for driver */

    void dump() volatile const {
        printf("PCI common config @%p:\n", this);
        printf("\tdevice feature select %#x features %#x\n", device_feature_select, device_feature);
        printf("\tdriver feature select %#x features %#x\n", driver_feature_select, driver_feature);
        printf("\tmsix vector %#x num queues %#x status %#x config gen %#x\n", config_msix_vector, num_queues,
            device_status, config_generation);
    }
};

STATIC_ASSERT(sizeof(virtio_pci_common_cfg) == 64);

void virtio_pci_bus::virtio_reset_device() {
    common_config()->device_status = 0;
    while (common_config()->device_status != 0)
        ;
}

void virtio_pci_bus::virtio_status_acknowledge_driver() {
    common_config()->device_status |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
}

void virtio_pci_bus::virtio_status_driver_ok() {
    common_config()->device_status |= VIRTIO_STATUS_DRIVER_OK;
}

uint32_t virtio_pci_bus::virtio_read_host_feature_word(uint32_t word) {
    common_config()->device_feature_select = word;
    return common_config()->device_feature;
}

void virtio_pci_bus::virtio_set_guest_features(uint32_t word, uint32_t features) {
    common_config()->driver_feature_select = word;
    common_config()->driver_feature = features;
}

void virtio_pci_bus::virtio_kick(uint16_t ring_index) {
    auto *notify = reinterpret_cast<volatile uint16_t *>(config_ptr(notify_cfg_) + static_cast<size_t>(ring_index * notify_offset_multiplier_));

    LTRACEF("notify ring %u ptr %p\n", ring_index, notify);

    *notify = ring_index;
    mb();
}

void virtio_pci_bus::register_ring(uint32_t page_size, uint32_t queue_sel, uint32_t queue_num, uint32_t queue_align, uint32_t queue_pfn) {
    auto *ccfg = common_config();

    // Using legacy split virtqueues packing strategy
    uint64_t ring_descriptor_paddr = static_cast<uint64_t>(queue_pfn) * page_size;
    uint64_t ring_available_paddr = ALIGN(ring_descriptor_paddr + static_cast<uint64_t>(16 * queue_num), 2);
    uint64_t ring_used_paddr = ALIGN(ring_available_paddr + 6 + static_cast<uint64_t>(2 * queue_num), queue_align);

    LTRACEF("queue %u size %u pfn %#x paddr (%#" PRIx64 ", %#" PRIx64 ", %#" PRIx64 ")\n",
             queue_sel, queue_num, queue_pfn, ring_descriptor_paddr, ring_available_paddr, ring_used_paddr);

    ccfg->queue_select = queue_sel;

    LTRACEF("existing queue_size %u\n", ccfg->queue_size);
    LTRACEF("existin notify off %u\n", ccfg->queue_notify_off);

    ccfg->queue_size = queue_num;
    ccfg->queue_desc = ring_descriptor_paddr;
    ccfg->queue_driver = ring_available_paddr;
    ccfg->queue_device = ring_used_paddr;
    ccfg->queue_enable = 1;
}

handler_return virtio_pci_bus::virtio_pci_irq(void *arg) {
    auto *bus = reinterpret_cast<virtio_pci_bus *>(arg);

    LTRACEF("dev %p, bus %p\n", bus->dev_, bus);

    volatile uint8_t *isr_status = bus->config_ptr(bus->isr_cfg_);
    LTRACEF("isr status register %p\n", isr_status);

    // reading status implicitly acks it and resets to 0
    uint32_t irq_status = *isr_status;
    LTRACEF("status %#x\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
        auto _ret = bus->dev_->handle_queue_interrupt();
        if (_ret == INT_RESCHEDULE) {
            ret = _ret;
        }

    }
    if (irq_status & 0x2) { /* config change */
        auto _ret = bus->dev_->handle_config_interrupt();
        if (_ret == INT_RESCHEDULE) {
            ret = _ret;
        }
    }

    LTRACEF("exiting irq\n");

    return ret;;
}

status_t virtio_pci_bus::init(virtio_device *dev, pci_location_t loc, size_t index) {
    LTRACE_ENTRY;

    DEBUG_ASSERT(!dev_ && dev);

    dev_ = dev;
    loc_ = loc;

    // read all of the capabilities for this virtio device
    bool map_bars[6] = {};
    for (size_t i = 0;; i++) {
        virtio_pci_cap cap;
        ssize_t err = pci_read_vendor_capability(loc, i, &cap, sizeof(cap));
        if (err < NO_ERROR || static_cast<size_t>(err) < sizeof(cap)) {
            break;
        }
        if (LOCAL_TRACE) dump_pci_cap(&cap);

        // save off the bar + range of all of the capabilities we care about
        virtio_pci_bus::config_pointer *cfg;
        switch (cap.cfg_type) {
            case VIRTIO_PCI_CAP_COMMON_CFG:
                cfg = &common_cfg_;
                goto common;
            case VIRTIO_PCI_CAP_NOTIFY_CFG: {
                // read in the extra 32bit offset multiplier
                virtio_pci_notify_cap cap2;
                err = pci_read_vendor_capability(loc, i, &cap2, sizeof(cap2));
                if (err < NO_ERROR || static_cast<size_t>(err) < sizeof(cap2)) {
                    break;
                }
                LTRACEF("notify offset multiplier %u\n", cap2.notify_off_multiplier);
                notify_offset_multiplier_ = cap2.notify_off_multiplier;

                cfg = &notify_cfg_;
            }
                goto common;
            case VIRTIO_PCI_CAP_ISR_CFG:
                cfg = &isr_cfg_;
                goto common;
            case VIRTIO_PCI_CAP_DEVICE_CFG:
                cfg = &device_cfg_;
                goto common;
            case VIRTIO_PCI_CAP_PCI_CFG:
                cfg = &pci_cfg_;
                // fallthrough
common:
                DEBUG_ASSERT(cfg);
                cfg->valid = true;
                cfg->bar = cap.bar;
                cfg->offset = cap.offset;
                cfg->length = cap.length;
                if (cap.bar < 6) {
                    map_bars[cap.bar] = true;
                }
        }
    }

    // check that at least the mandatory capabilities are present
    if (!(common_cfg_.valid && notify_cfg_.valid && isr_cfg_.valid && pci_cfg_.valid)) {
        return ERR_NOT_FOUND;
    }

    // map in the bars we care about
    pci_bar_t bars[6];
    status_t err = pci_bus_mgr_read_bars(loc, bars);
    if (err != NO_ERROR) {
        return err;
    }

    LTRACEF("virtio-pci BARS:\n");
    if (LOCAL_TRACE) pci_dump_bars(bars, 6);

    for (int i = 0; i < 6; i++) {
        if (map_bars[i] && !bars[i].io) {
            auto &bar_map = bar_map_[i];
#if WITH_KERNEL_VM
            char str[32];
            snprintf(str, sizeof(str), "virtio%zu bar%d", index, i);
            err = vmm_alloc_physical(vmm_get_kernel_aspace(), str, bars[i].size, reinterpret_cast<void **>(&bar_map.vaddr), 0,
                                     bars[i].addr, /* vmm_flags */ 0, ARCH_MMU_FLAG_UNCACHED_DEVICE);
            if (err != NO_ERROR) {
                printf("error mapping bar %d\n", i);
                continue;
            }
            bar_map.mapped = true;
            LTRACEF("bar %d mapped at %p\n", i, bar_map.vaddr);
#else
            // no need to map, it's already available at the physical address
            if (sizeof(void *) < 8 && (bars[i].addr + bars[i].size) > UINT32_MAX) {
                TRACEF("aborting due to 64bit BAR on 32bit arch\n");
                return ERR_NO_MEMORY;
            }
            bar_map.vaddr = (uint8_t *)(uintptr_t)bars[i].addr;
#endif
        }
    }

    // enable the device
    pci_bus_mgr_enable_device(loc);

    // look at the common configuration
    if (LOCAL_TRACE) common_config()->dump();

    // read the device-independent feature bits
    uint64_t features = virtio_read_host_feature_word_64(0);
    virtio_dump_device_independent_features_bits(features);

    // accept mandatory features
    if (features & VIRTIO_F_VERSION_1) {
        virtio_set_guest_features(1, VIRTIO_F_VERSION_1 >> 32);
    }

    uint irq_base;
    err = pci_bus_mgr_allocate_msi(loc_, 1, &irq_base);
    if (err != NO_ERROR) {
        // fall back to regular IRQs
        err = pci_bus_mgr_allocate_irq(loc_, &irq_base);
        if (err != NO_ERROR) {
            printf("block: unable to allocate IRQ\n");
            return err;
        }
        ::mask_interrupt(irq_base);
        register_int_handler(irq_base, virtio_pci_irq, this);
    } else {
        ::mask_interrupt(irq_base);
        register_int_handler_msi(irq_base, virtio_pci_irq, this, true);
    }
    set_irq(irq_base);
    LTRACEF("IRQ number %#x\n", irq_base);

    return NO_ERROR;
}

static status_t init_block(pci_location_t loc, const virtio_pci_devices &dev_table_entry, size_t index) {
    LTRACE_ENTRY;

#if WITH_DEV_VIRTIO_BLOCK
    // create a virtio_pci_bus object and initialize it based on the location
    auto *bus = new virtio_pci_bus();
    auto *dev = new virtio_device(bus);

    auto err = bus->init(dev, loc, index);
    if (err != NO_ERROR) {
        delete bus;
        return err;
    }

    // TODO: move the config pointer getter that devices use into the bus
    dev->set_config_ptr(bus->device_config());

    err = virtio_block_init(dev, bus->virtio_read_host_feature_word(0));
    if (err != NO_ERROR) {
        PANIC_UNIMPLEMENTED;
    }

    return err;
#else
    return ERR_NOT_FOUND;
#endif
}

static void virtio_pci_init(uint level) {
    LTRACE_ENTRY;

    for (auto &dev: devices) {
        for (size_t i = 0; ; i++) {
            pci_location_t loc;
            status_t err = pci_bus_mgr_find_device(&loc, dev.device_id, 0x1af4, i);
            if (err != NO_ERROR) {
                break;
            }

            char str[14];
            printf("virtio-pci: looking at device at %s\n", pci_loc_string(loc, str));

            // call the init routine
            if (dev.init) {
                dev.init(loc, dev, i);
            }
        }
    }

    LTRACE_EXIT;
}

LK_INIT_HOOK(virtio_pci, &virtio_pci_init, LK_INIT_LEVEL_PLATFORM + 1);

#endif

