/*
 * Copyright (c) 2021 Travis Geiseblrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <sys/types.h>
#include <dev/bus/pci.h>
#include <lk/cpp.h>
#include <lk/err.h>
#include <lk/list.h>

namespace pci {

class bus;
struct capability;

// generic pci device
class device {
public:
    device(pci_location_t loc, bus *bus);
    virtual ~device();

    DISALLOW_COPY_ASSIGN_AND_MOVE(device);

    static status_t probe(pci_location_t loc, bus *bus, device **out_device);

    status_t probe_capabilities();
    status_t init_msi_capability(capability *cap);
    status_t init_msix_capability(capability *cap);

    status_t allocate_irq(uint *irq);
    status_t allocate_msi(size_t num_requested, uint *msi_base);
    status_t load_config();
    status_t load_bars();

    status_t enable();

    // ask the device to add up the sizes of all its bars and return the sum
    // bridges will be expected to recurse into sub-bridges
    struct bar_sizes {
        uint32_t io_size;
        uint32_t mmio_size;
        uint64_t mmio64_size;
        uint64_t prefetchable_size;
        uint64_t prefetchable64_size;

        uint8_t io_align;
        uint8_t mmio_align;
        uint8_t mmio64_align;
        uint8_t prefetchable_align;
        uint8_t prefetchable64_align;

        bar_sizes &operator+=(const bar_sizes &a);
    };
    virtual status_t compute_bar_sizes(bar_sizes *sizes);

    struct bar_alloc_request {
        // linked list node
        list_node node;

        // requst for allocation of this type
        pci_resource_type type;
        uint64_t size;
        uint8_t align; // power of 2

        bool bridge; // either a bridge request or a bar
        bool prefetchable; // prefetchable request (only makes sense for mmio or mmio64)

        device *dev;
        uint8_t bar_num;

        void dump();
    };
    virtual status_t get_bar_alloc_requests(list_node *bar_alloc_requests);
    virtual status_t assign_resource(bar_alloc_request *request, uint64_t address);
    virtual status_t assign_child_resources() { return NO_ERROR; }

    pci_location_t loc() const { return loc_; }
    const bus *get_bus() const { return bus_; }

    uint16_t device_id() const { return config_.device_id; }
    uint16_t vendor_id() const { return config_.vendor_id; }
    uint8_t base_class() const { return config_.base_class; }
    uint8_t sub_class() const { return config_.sub_class; }
    uint8_t interface() const { return config_.program_interface; }
    uint8_t header_type() const { return config_.header_type & PCI_HEADER_TYPE_MASK; }

    status_t read_bars(pci_bar_t bar[6]);
    ssize_t read_vendor_capability(size_t index, void *buf, size_t buflen);

    bool has_msi() const { return msi_cap_; }
    bool has_msix() const { return msix_cap_; }

    virtual void dump(size_t indent = 0);

protected:
    // let the bus device directly manipulate our list node
    friend class bus;
    list_node node = LIST_INITIAL_CLEARED_VALUE;

    pci_location_t loc_ = {};
    bus *bus_ = nullptr;

    pci_config_t config_ = {};
    pci_bar_t bars_[6] = {};

    // capability list
    list_node capability_list_ = LIST_INITIAL_VALUE(capability_list_);
    capability *msi_cap_ = nullptr;
    capability *msix_cap_ = nullptr;
};

struct capability {
    list_node node = LIST_INITIAL_CLEARED_VALUE;
    uint16_t config_offset = 0;
    uint16_t id = 0;

    // simple accessors
    bool is_msi() const { return id == 0x5; }
    bool is_msix() const { return id == 0x11; }
};

inline device::bar_sizes operator+(const device::bar_sizes &a, const device::bar_sizes &b) {
    device::bar_sizes result;

    result.io_size = a.io_size + b.io_size;
    result.mmio_size = a.mmio_size + b.mmio_size;
    result.mmio64_size = a.mmio64_size + b.mmio64_size;
    result.prefetchable_size = a.prefetchable_size + b.prefetchable_size;
    result.prefetchable64_size = a.prefetchable64_size + b.prefetchable64_size;

    result.io_align = (a.io_align > b.io_align) ? a.io_align : b.io_align;
    result.mmio_align = (a.mmio_align > b.mmio_align) ? a.mmio_align : b.mmio_align;
    result.mmio64_align = (a.mmio64_align > b.mmio64_align) ? a.mmio64_align : b.mmio64_align;
    result.prefetchable_align = (a.prefetchable_align > b.prefetchable_align) ? a.prefetchable_align : b.prefetchable_align;
    result.prefetchable64_align = (a.prefetchable64_align > b.prefetchable64_align) ? a.prefetchable64_align : b.prefetchable64_align;

    return result;
}

inline device::bar_sizes &device::bar_sizes::operator+=(const device::bar_sizes &a) {
    io_size += a.io_size;
    mmio_size += a.mmio_size;
    mmio64_size += a.mmio64_size;
    prefetchable_size += a.prefetchable_size;
    prefetchable64_size += a.prefetchable64_size;

    io_align = (io_align > a.io_align) ? io_align : a.io_align;
    mmio_align = (mmio_align > a.mmio_align) ? mmio_align : a.mmio_align;
    mmio64_align = (mmio64_align > a.mmio64_align) ? mmio64_align : a.mmio64_align;
    prefetchable_align = (prefetchable_align > a.prefetchable_align) ? prefetchable_align : a.prefetchable_align;
    prefetchable64_align = (prefetchable64_align > a.prefetchable64_align) ? prefetchable64_align : a.prefetchable64_align;

    return *this;
}

} // pci
