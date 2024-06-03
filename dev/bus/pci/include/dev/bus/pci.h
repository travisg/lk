/*
 * Copyright (c) 2009 Corey Tabaka
 * Copyright (c) 2020 Travis Geiseblrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <assert.h>
#include <sys/types.h>
#include <lk/compiler.h>

// pci level structures and defines
#include <hw/pci.h>

__BEGIN_CDECLS

/*
 * PCI address structure
 */
typedef struct {
    uint16_t segment;
    uint8_t bus;
    uint8_t dev;
    uint8_t fn;
} pci_location_t;

typedef struct {
    uint64_t addr;
    size_t size;
    bool io;
    bool prefetchable;
    bool size_64;
    bool valid;
} pci_bar_t;

// only use one of these two:
// try to detect PCI based on legacy PC PCI accessor methods
status_t pci_init_legacy(void);

// try to detect PCI based on a known ecam base.
status_t pci_init_ecam(paddr_t ecam_base, uint16_t segment, uint8_t start_bus, uint8_t end_bus);

// user facing C api
int pci_get_last_bus(void);
int pci_get_last_segment(void);

status_t pci_read_config(pci_location_t loc, pci_config_t *config);

status_t pci_read_config_byte(pci_location_t state, uint32_t reg, uint8_t *value);
status_t pci_read_config_half(pci_location_t state, uint32_t reg, uint16_t *value);
status_t pci_read_config_word(pci_location_t state, uint32_t reg, uint32_t *value);

status_t pci_write_config_byte(pci_location_t state, uint32_t reg, uint8_t value);
status_t pci_write_config_half(pci_location_t state, uint32_t reg, uint16_t value);
status_t pci_write_config_word(pci_location_t state, uint32_t reg, uint32_t value);

// pci bus manager
// builds a list of devices and allows for various operations on the list

// C level visitor routine
typedef void(*pci_visit_routine)(pci_location_t loc, void *cookie);
status_t pci_bus_mgr_visit_devices(pci_visit_routine routine, void *cookie);

// must be called before pci_bus_mgr_init if available
enum pci_resource_type {
    PCI_RESOURCE_IO_RANGE = 0,
    PCI_RESOURCE_MMIO_RANGE,
    PCI_RESOURCE_MMIO64_RANGE,
};
status_t pci_bus_mgr_add_resource(enum pci_resource_type, uint64_t mmio_base, uint64_t len);

status_t pci_bus_mgr_assign_resources(void);

// must be called after pci_init_*();
status_t pci_bus_mgr_init(void);

// Look for the Nth match of device id and vendor id.
// Either device or vendor is skipped if set to 0xffff.
// Error if both is set to 0xffff.
status_t pci_bus_mgr_find_device(pci_location_t *state, uint16_t device_id, uint16_t vendor_id, size_t index);

// Look for the Nth match of combination of base, subclass, and interface.
// interface and subclass may be set to 0xff in which case it will skip.
status_t pci_bus_mgr_find_device_by_class(pci_location_t *state, uint8_t base_class, uint8_t subclass, uint8_t interface, size_t index);

// set io and mem enable on the device
status_t pci_bus_mgr_enable_device(const pci_location_t loc);

// read a list of up to 6 bars out of the device. each is marked with a valid bit
status_t pci_bus_mgr_read_bars(const pci_location_t loc, pci_bar_t bar[6]);

// try to allocate one or more msi vectors for this device
status_t pci_bus_mgr_allocate_msi(const pci_location_t loc, size_t num_requested, uint *irqbase);

// try to allocate one or more msi-x vectors for this device
status_t pci_bus_mgr_allocate_msix(const pci_location_t loc, size_t num_requested, uint *irqbase);

// allocate a regular irq for this device and return it in irqbase
status_t pci_bus_mgr_allocate_irq(const pci_location_t loc, uint *irqbase);

// XXX sort this nicely
ssize_t pci_read_vendor_capability(const pci_location_t loc, size_t index, void *buf, size_t buflen);

// return a pointer to a formatted string
const char *pci_loc_string(pci_location_t loc, char out_str[14]);

// debug printing routines
void pci_dump_bar(const pci_bar_t *bar, int index);
void pci_dump_bars(pci_bar_t bar[6], size_t count);
const char *pci_resource_type_to_str(enum pci_resource_type);

__END_CDECLS

#if __cplusplus

// C++ helper routine for pci_bus_mgr_visit_devices
// Wrapper to convert lambdas and other function like things to the C api
template <typename T>
void pci_bus_mgr_visit_devices(T routine) {
    struct vdata {
        T &routine;
    };

    auto v = [](pci_location_t loc, void *cookie) {
        vdata *data = static_cast<vdata *>(cookie);
        data->routine(loc);
    };

    vdata data = { routine };
    pci_bus_mgr_visit_devices(v, &data);
}

inline bool operator==(pci_location_t a, pci_location_t b) {
    return a.segment == b.segment &&
        a.bus == b.bus &&
        a.dev == b.dev &&
        a.fn == b.fn;
}

#endif // __cplusplus

