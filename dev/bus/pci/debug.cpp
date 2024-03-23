/*
 * Copyright (c) 2009 Corey Tabaka
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <app.h>
#include <lk/debug.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <lk/compiler.h>
#include <lk/err.h>
#include <platform.h>
#include <dev/bus/pci.h>
#include <lk/console_cmd.h>

/*
 * enumerates pci devices
 */
static void pci_list(void) {
    pci_location_t state;
    int busses = 0, devices = 0, lines = 0, ret;

    printf("Scanning (brute force)...\n");

    for (int segment = 0; segment <= (int)pci_get_last_segment(); segment++) {
        state.segment = segment;

        for (int bus = 0; bus <= (int)pci_get_last_bus(); bus++) {
            state.bus = bus;
            busses++;

            for (int dev = 0; dev < 32; dev++) {
                state.dev = dev;

                for (int fn = 0; fn < 8; fn++) {
                    state.fn = fn;

                    uint16_t vendor_id;
                    ret = pci_read_config_half(state, PCI_CONFIG_VENDOR_ID, &vendor_id);
                    if (ret != NO_ERROR) goto error;
                    if (vendor_id == 0xffff && fn == 0) {
                        // skip this device now before bothering to read the rest of the
                        // configuration in.
                        break;
                    }

                    pci_config_t config;
                    ret = pci_read_config(state, &config);
                    if (ret != NO_ERROR) continue;

                    if (config.vendor_id != 0xffff) {
                        printf("%04x:%02x:%02x.%0x vendor_id=%04x device_id=%04x, header_type=%02x "
                                "base_class=%02x, sub_class=%02x, interface=%02x, irq_line=%u, irq_pin=%u\n",
                               state.segment, state.bus, state.dev, state.fn,
                               config.vendor_id, config.device_id, config.header_type, config.base_class,
                               config.sub_class, config.program_interface, config.type0.interrupt_line,
                               config.type0.interrupt_pin);
                        devices++;
                        lines++;
                    }

                    if ((fn == 0) && ~config.header_type & PCI_HEADER_TYPE_MULTI_FN) {
                        // this is not a multi-function device, so advance to the next device
                        // only check when looking at function 0 of a device
                        break;
                    }
                }
            }
        }
    }

    printf("... done. Scanned %d busses, found %d device/functions\n", busses, devices);
quit:
    return;

error:
    printf("Error while reading PCI config space: %02x\n", ret);
}

/*
 * a somewhat fugly pci config space examine/modify command. this should probably
 * be broken up a bit.
 */
static int pci_config(int argc, const console_cmd_args *argv) {
    pci_location_t loc;
    pci_config_t config;
    uint32_t offset;
    unsigned int i;
    int ret;

    if (argc < 6) {
        return -1;
    }

    if (!strcmp(argv[2].str, "dump")) {
        loc.segment = 0;
        loc.bus = atoui(argv[3].str);
        loc.dev = atoui(argv[4].str);
        loc.fn = atoui(argv[5].str);

        if (pci_read_config(loc, &config) != NO_ERROR) {
            printf("error reading configuration\n");
            return -1;
        }

        printf("Device at %04x:%02x:%02x.%1x vendor id=%04x device id=%04x\n", loc.segment, loc.bus,
               loc.dev, loc.fn, config.vendor_id, config.device_id);
        printf("command=%04x status=%04x pi=%02x sub cls=%02x base cls=%02x htype=%02x\n",
               config.command, config.status, config.program_interface,
               config.sub_class, config.base_class, config.header_type & PCI_HEADER_TYPE_MASK);
        printf("int pin %u int line %u\n", config.type0.interrupt_pin, config.type0.interrupt_line);

        uint8_t header_type = config.header_type & PCI_HEADER_TYPE_MASK;
        if (header_type == PCI_HEADER_TYPE_STANDARD) { // type 0
            for (i=0; i < 6; i+=2) {
                printf("bar%d=%08x  bar%d=%08x\n", i, config.type0.base_addresses[i],
                       i+1, config.type0.base_addresses[i+1]);
            }
        } else if (header_type == PCI_HEADER_TYPE_PCI_BRIDGE) { // type 1
            printf("primary bus=%02x secondary=%02x subordinate=%02x\n",
                   config.type1.primary_bus, config.type1.secondary_bus, config.type1.subordinate_bus);
            for (i=0; i < 2; i+=2) {
                printf("bar%d=%08x  bar%d=%08x\n", i, config.type1.base_addresses[i],
                       i+1, config.type1.base_addresses[i+1]);
            }
            printf("mem base=%08x mem limit=%08x\n", config.type1.memory_base << 16,
                   (config.type1.memory_limit << 16) | 0xf'ffff);
            switch ((config.type1.io_base & 0xf)) {
                case 0: // 16 bit io addressing
                    printf("io base=%04x io limit=%04x\n", (config.type1.io_base & 0xf0) << 8,
                           ((config.type1.io_limit & 0xf0) << 8) | 0xfff );
                    break;
                case 1: // 32 bit io addressing
                    printf("io base=%08x io limit=%08x\n",
                           (config.type1.io_base % 0xf0) << 8 | config.type1.io_base_upper << 16,
                           ((config.type1.io_limit & 0xf0) << 8) | 0xfff | config.type1.io_limit_upper << 16);
                    break;
            }
            switch ((config.type1.prefetchable_memory_base & 0xf)) {
                case 0: // 32 bit prefetchable addressing
                    printf("prefetchable base=%08x prefetchable limit=%08x\n",
                           (config.type1.prefetchable_memory_base & 0xfff0) << 16,
                           (config.type1.prefetchable_memory_limit & 0xfff0) << 16 | 0xf'ffff);
                    break;
                case 1: // 64 bit prefetchable addressing
                    printf("prefetchable base=%llx prefetchable limit=%llx\n",
                           ((uint64_t)config.type1.prefetchable_memory_base & 0xfff0) << 16 |
                           ((uint64_t)config.type1.prefetchable_base_upper << 32),
                           ((uint64_t)config.type1.prefetchable_memory_limit & 0xfff0) << 16 | 0xf'ffff |
                           ((uint64_t)config.type1.prefetchable_limit_upper << 32));
                    break;

            }
        }
        hexdump8_ex(&config, sizeof(config), 0);
    } else if (!strcmp(argv[2].str, "hexdump")) {
        loc.segment = 0;
        loc.bus = atoui(argv[3].str);
        loc.dev = atoui(argv[4].str);
        loc.fn = atoui(argv[5].str);

        if (pci_read_config(loc, &config) != NO_ERROR) {
            printf("error reading configuration\n");
            return -1;
        }

        hexdump8_ex(&config, sizeof(config), 0);
    } else if (!strcmp(argv[2].str, "rb") || !strcmp(argv[2].str, "rh") || !strcmp(argv[2].str, "rw")) {
        if (argc != 7) {
            return -1;
        }

        loc.segment = 0;
        loc.bus = atoui(argv[3].str);
        loc.dev = atoui(argv[4].str);
        loc.fn = atoui(argv[5].str);
        offset = atoui(argv[6].str);

        switch (argv[2].str[1]) {
            case 'b': {
                uint8_t value;
                ret = pci_read_config_byte(loc, offset, &value);
                if (ret != NO_ERROR) goto error;

                printf("byte at device %04x:%02x:%02x.%1x config offset %04x: %02x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);
            }
            break;

            case 'h': {
                uint16_t value;
                ret = pci_read_config_half(loc, offset, &value);
                if (ret != NO_ERROR) goto error;

                printf("half at device %04x:%02x:%02x.%1x config offset %04x: %04x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);
            }
            break;

            case 'w': {
                uint32_t value;
                ret = pci_read_config_word(loc, offset, &value);
                if (ret != NO_ERROR) goto error;

                printf("word at device %04x:%02x:%02x.%1x config offset %04x: %08x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);
            }
            break;
        }
    } else if (!strcmp(argv[2].str, "mb") || !strcmp(argv[2].str, "mh") || !strcmp(argv[2].str, "mw")) {
        if (argc != 8) {
            return -1;
        }

        loc.segment = 0;
        loc.bus = atoui(argv[3].str);
        loc.dev = atoui(argv[4].str);
        loc.fn = atoui(argv[5].str);
        offset = atoui(argv[6].str);

        switch (argv[2].str[1]) {
            case 'b': {
                uint8_t value = atoui(argv[7].str);
                ret = pci_write_config_byte(loc, offset, value);
                if (ret != NO_ERROR) goto error;

                printf("byte to device %04x:%02x:%02x.%1x config offset %04x: %02x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);
            }
            break;

            case 'h': {
                uint16_t value = atoui(argv[7].str);
                ret = pci_write_config_half(loc, offset, value);
                if (ret != NO_ERROR) goto error;

                printf("half to device %04x:%02x:%02x.%1x config offset %04x: %04x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);

            }
            break;

            case 'w': {
                uint32_t value = atoui(argv[7].str);
                ret = pci_write_config_word(loc, offset, value);
                if (ret != NO_ERROR) goto error;

                printf("word to device %04x:%02x:%02x.%1x config offset %04x: %08x\n", loc.segment, loc.bus, loc.dev, loc.fn, offset, value);
            }
            break;
        }
    } else {
        return -1;
    }

    return 0;

error:
    printf("Error while reading PCI config space: %02x\n", ret);
    return -2;
}

static int pci_cmd(int argc, const console_cmd_args *argv) {
    if (argc < 2) {
        printf("pci commands:\n");
usage:
        printf("%s list\n", argv[0].str);
        printf("%s config dump <bus> <dev> <fn>\n", argv[0].str);
        printf("%s config hexdump <bus> <dev> <fn>\n", argv[0].str);
        printf("%s config <rb|rh|rw> <bus> <dev> <fn> <offset>\n", argv[0].str);
        printf("%s config <mb|mh|mw> <bus> <dev> <fn> <offset> <value>\n", argv[0].str);
        goto out;
    }

    if (!strcmp(argv[1].str, "list")) {
        pci_list();
    } else if (!strcmp(argv[1].str, "config")) {
        if (pci_config(argc, argv)) {
            goto usage;
        }
    } else {
        goto usage;
    }

out:
    return 0;
}

STATIC_COMMAND_START
STATIC_COMMAND("pci", "pci toolbox", &pci_cmd)
STATIC_COMMAND_END(pcitests);

