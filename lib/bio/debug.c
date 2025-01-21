/*
 * Copyright (c) 2009-2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <assert.h>
#include <lk/debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <lk/err.h>
#include <lk/console_cmd.h>
#include <lib/bio.h>
#include <platform.h>
#include <kernel/thread.h>

#if WITH_LIB_CKSUM
#include <lib/cksum.h>
#endif
#if WITH_LIB_PARTITION
#include <lib/partition.h>
#endif

#define DMA_ALIGNMENT (CACHE_LINE)
#define THREE_BYTE_ADDR_BOUNDARY (16777216)
#define SUB_ERASE_TEST_SAMPLES (32)

#if LK_DEBUGLEVEL > 0
static int cmd_bio(int argc, const console_cmd_args *argv);
static int bio_test_device(bdev_t *device);

STATIC_COMMAND_START
STATIC_COMMAND("bio", "block io debug commands", &cmd_bio)
STATIC_COMMAND_END(bio);

static int cmd_bio(int argc, const console_cmd_args *argv) {
    int rc = 0;

    if (argc < 2) {
notenoughargs:
        printf("not enough arguments:\n");
usage:
        printf("%s list\n", argv[0].str);
        printf("%s read <device> <address> <offset> <len>\n", argv[0].str);
        printf("%s write <device> <address> <offset> <len>\n", argv[0].str);
        printf("%s dump <device> <offset> <len>\n", argv[0].str);
        printf("%s erase <device> <offset> <len>\n", argv[0].str);
        printf("%s ioctl <device> <request> <arg>\n", argv[0].str);
        printf("%s remove <device>\n", argv[0].str);
        printf("%s test <device>\n", argv[0].str);
#if WITH_LIB_PARTITION
        printf("%s partscan <device> [offset]\n", argv[0].str);
#endif
#if WITH_LIB_CKSUM
        printf("%s crc32 <device> <offset> <len> [repeat]\n", argv[0].str);
#endif
        return -1;
    }

    if (!strcmp(argv[1].str, "list")) {
        bio_dump_devices();
    } else if (!strcmp(argv[1].str, "read")) {
        if (argc < 6) goto notenoughargs;

        addr_t address = argv[3].u;
        off_t offset = argv[4].u; // XXX use long
        size_t len = argv[5].u;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        lk_time_t t = current_time();
        ssize_t err = bio_read(dev, (void *)address, offset, len);
        t = current_time() - t;
        dprintf(INFO, "bio_read returns %d, took %u msecs (%d bytes/sec)\n", (int)err, (uint)t, (uint32_t)((uint64_t)err * 1000 / t));

        bio_close(dev);

        rc = err;
    } else if (!strcmp(argv[1].str, "write")) {
        if (argc < 6) goto notenoughargs;

        addr_t address = argv[3].u;
        off_t offset = argv[4].u; // XXX use long
        size_t len = argv[5].u;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        lk_time_t t = current_time();
        ssize_t err = bio_write(dev, (void *)address, offset, len);
        t = current_time() - t;
        dprintf(INFO, "bio_write returns %d, took %u msecs (%d bytes/sec)\n", (int)err, (uint)t, (uint32_t)((uint64_t)err * 1000 / t));

        bio_close(dev);

        rc = err;
    } else if (!strcmp(argv[1].str, "dump")) {
        if (argc < 5) {
            printf("not enough arguments:\n");
            goto usage;
        }

        off_t offset = argv[3].u; // XXX use long
        size_t len = argv[4].u;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        uint8_t *buf = memalign(CACHE_LINE, 256);
        ssize_t err = 0;
        while (len > 0) {
            size_t  amt = MIN(256, len);
            ssize_t err_len = bio_read(dev, buf, offset, amt);

            if (err_len < 0) {
                dprintf(ALWAYS, "read error %s %zu@%zu (err_len %zd)\n",
                        argv[2].str, amt, (size_t)offset, err_len);
                break;
            }

            DEBUG_ASSERT((size_t)err_len <= amt);
            hexdump8_ex(buf, err_len, offset);

            if ((size_t)err_len != amt) {
                dprintf(ALWAYS, "short read from %s @%zu (wanted %zu, got %zu)\n",
                        argv[2].str, (size_t)offset, amt, (size_t)err);
                break;
            }

            offset += amt;
            len    -= amt;
        }

        bio_close(dev);
        rc = err;
    } else if (!strcmp(argv[1].str, "erase")) {
        if (argc < 5) goto notenoughargs;

        off_t offset = argv[3].u; // XXX use long
        size_t len = argv[4].u;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        lk_time_t t = current_time();
        ssize_t err = bio_erase(dev, offset, len);
        t = current_time() - t;
        dprintf(INFO, "bio_erase returns %d, took %u msecs (%d bytes/sec)\n", (int)err, (uint)t, (uint32_t)((uint64_t)err * 1000 / t));

        bio_close(dev);

        rc = err;
    } else if (!strcmp(argv[1].str, "ioctl")) {
        if (argc < 4) goto notenoughargs;

        int request = argv[3].u;
        unsigned long arg = (argc == 5) ? argv[4].u : 0;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        int err = bio_ioctl(dev, request, (void *)arg);
        dprintf(INFO, "bio_ioctl returns %d\n", err);

        bio_close(dev);

        rc = err;
    } else if (!strcmp(argv[1].str, "remove")) {
        if (argc < 3) goto notenoughargs;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        bio_unregister_device(dev);
        bio_close(dev);
    } else if (!strcmp(argv[1].str, "test")) {
        if (argc < 3) goto notenoughargs;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        int err = bio_test_device(dev);
        bio_close(dev);

        rc = err;
#if WITH_LIB_PARTITION
    } else if (!strcmp(argv[1].str, "partscan")) {
        if (argc < 3) goto notenoughargs;

        off_t offset = 0;
        if (argc > 3)
            offset = argv[3].u;

        rc = partition_publish(argv[2].str, offset);
        dprintf(INFO, "partition_publish returns %d\n", rc);
#endif
#if WITH_LIB_CKSUM
    } else if (!strcmp(argv[1].str, "crc32")) {
        if (argc < 5) goto notenoughargs;

        unsigned long offset = argv[3].u;
        size_t len = argv[4].u;

        bdev_t *dev = bio_open(argv[2].str);
        if (!dev) {
            printf("error opening block device\n");
            return -1;
        }

        void *buf = malloc(dev->block_size);

        bool repeat = false;
        if (argc >= 6 && !strcmp(argv[5].str, "repeat")) {
            repeat = true;
        }

        do {
            ulong crc = 0;
            unsigned long pos = offset;
            while (pos < offset + len) {
                ssize_t err = bio_read(dev, buf, pos, MIN(len - (pos - offset), dev->block_size));

                if (err <= 0) {
                    printf("error reading at offset 0x%lx\n", offset + pos);
                    break;
                }

                crc = crc32(crc, buf, err);

                pos += err;
            }

            printf("crc 0x%08lx\n", crc);
        } while (repeat);

        bio_close(dev);
        free(buf);

#endif
    } else {
        printf("unrecognized subcommand\n");
        goto usage;
    }

    return rc;
}

#endif

// Returns the number of blocks that do not match the reference pattern.
static bool is_valid_block(bdev_t *device, bnum_t block_num, uint8_t *pattern,
                           size_t pattern_length) {
    uint8_t *block_contents = memalign(DMA_ALIGNMENT, device->block_size);

    ssize_t n_bytes = device->read_block(device, block_contents, block_num, 1);
    if (n_bytes < 0 || n_bytes != (ssize_t)device->block_size) {
        free(block_contents);
        return false;
    }

    for (size_t i = 0; i < device->block_size; i++) {
        if (block_contents[i] != pattern[i % pattern_length]) {
            free(block_contents);
            block_contents = NULL;
            return false;
        }
    }

    free(block_contents);
    block_contents = NULL;

    return true;
}

static ssize_t erase_test(bdev_t *device) {
    printf("erasing device...\n");

    ssize_t err = bio_erase(device, 0, device->total_size);
    if (err < 0) {
        return err;
    } else if (err != device->total_size) {
        return ERR_IO;
    }

    printf("validating erase...\n");
    size_t num_invalid_blocks = 0;
    for (bnum_t bnum = 0; bnum < device->block_count; bnum++) {
        if (!is_valid_block(device, bnum, &device->erase_byte, sizeof(device->erase_byte))) {
            num_invalid_blocks++;
        }
    }
    return num_invalid_blocks;
}

static bool test_erase_block(bdev_t *device, uint32_t block_addr) {
    bool success = false;
    uint8_t valid_byte[1];

    uint8_t *block_contents = memalign(DMA_ALIGNMENT, device->block_size);
    memset(block_contents, ~(device->erase_byte), device->block_size);

    ssize_t err = bio_write_block(device, block_contents, block_addr, 1);
    if (err != (ssize_t)device->block_size) {
        goto finish;
    }

    valid_byte[0] = ~(device->erase_byte);
    if (!is_valid_block(device, block_addr, valid_byte, 1)) {
        goto finish;
    }

    err = bio_erase(device, block_addr * device->block_size, 1);
    if (err <= 0) {
        goto finish;
    }

    valid_byte[0] = device->erase_byte;
    if (is_valid_block(device, block_addr, valid_byte, 1)) {
        success = true;
    }

finish:
    free(block_contents);
    return success;
}

// Ensure that (sub)sector erase work.
static bool sub_erase_test(bdev_t *device, uint32_t n_samples) {
    printf("Sampling the device %d times.\n", n_samples);
    for (uint32_t i = 0; i < n_samples; i++) {
        bnum_t block_addr = rand() % device->block_count;
        if (!test_erase_block(device, block_addr)) {
            return false;
        }
    }
    return true;
}

static uint8_t get_signature(uint32_t word) {
    uint8_t *sigptr = (uint8_t *)(&word);
    return sigptr[0] ^ sigptr[1] ^ sigptr[2] ^ sigptr[3];
}

// returns the number of blocks where the write was not successful.
static ssize_t write_test(bdev_t *device) {
    uint8_t *test_buffer = memalign(DMA_ALIGNMENT, device->block_size);

    for (bnum_t bnum = 0; bnum < device->block_count; bnum++) {
        memset(test_buffer, get_signature(bnum), device->block_size);
        ssize_t err = bio_write_block(device, test_buffer, bnum, 1);
        if (err < 0) {
            free(test_buffer);
            return err;
        }
    }

    size_t num_errors = 0;
    uint8_t expected_pattern[1];
    for (bnum_t bnum = 0; bnum < device->block_count; bnum++) {
        expected_pattern[0] = get_signature(bnum);
        if (!is_valid_block(device, bnum, expected_pattern, sizeof(expected_pattern))) {
            num_errors++;
        }
    }

    free(test_buffer);

    return num_errors;
}

static status_t memory_mapped_test(bdev_t *device) {
    status_t retcode = NO_ERROR;

    uint8_t *test_buffer = memalign(DMA_ALIGNMENT, device->block_size);
    if (!test_buffer) {
        printf("Could not allocate %zu bytes for a temporary buffer. "
               "Aborting.\n", device->block_size);
        return ERR_NO_MEMORY;
    }

    uint8_t *reference_buffer = memalign(DMA_ALIGNMENT, device->block_size);
    if (!reference_buffer) {
        printf("Could not allocate %zu bytes for a temporary reference "
               "buffer. Aborting.\n", device->block_size);
        free(test_buffer);
        return ERR_NO_MEMORY;
    }

    // Erase the first page of the Device.
    ssize_t err = bio_erase(device, 0, device->block_size);
    if (err < (ssize_t)device->block_size) {
        printf("Expected to erase at least %zu bytes but only erased %zd. "
               "Not continuing to test memory mapped mode.\n",
               device->block_size, err);
        retcode = ERR_IO;
        goto finish;
    }

    // Write a pattern to the first page of the device.
    uint8_t pattern_seed = (uint8_t)(rand() % 256);

    for (size_t i = 0; i < device->block_size; i++) {
        test_buffer[i] = (uint8_t)((pattern_seed + i) % 256);
    }

    err = bio_write_block(device, test_buffer, 0, 1);
    if (err != (ssize_t)device->block_size) {
        printf("Error while writing test pattern to device. Expected to write "
               "%zu bytes but actually wrote %zd. Not continuing to test memory "
               "mapped mode.\n", device->block_size, err);
        retcode = ERR_IO;
        goto finish;
    }

    // Put the device into linear mode if possible.
    uint8_t *devaddr;
    int ioctl_result = bio_ioctl(device, BIO_IOCTL_GET_MEM_MAP, (void *)&devaddr);
    if (ioctl_result == ERR_NOT_SUPPORTED) {
        printf("Device does not support linear mode. Aborting.\n");
        retcode = ERR_NOT_SUPPORTED;
        goto finish;
    } else if (ioctl_result != NO_ERROR) {
        printf("BIO_IOCTL_GET_MEM_MAP returned error %d. Aborting.\n",
               ioctl_result);
        retcode = ioctl_result;
        goto finish;
    }

    uint8_t *testptr = test_buffer;
    for (uint i = 0; i < device->block_size; i++) {
        if (*testptr != *devaddr) {
            printf("Data mismatch at position %d. Expected %d got %d. "
                   "Aborting.\n", i, *testptr, *devaddr);
            goto finish;
        }
        testptr++;
        devaddr++;
    }

    // Put the device back into command mode.
    ioctl_result = bio_ioctl(device, BIO_IOCTL_PUT_MEM_MAP, NULL);
    if (ioctl_result != NO_ERROR) {
        printf("BIO_IOCTL_GET_MEM_MAP returned error %d. Aborting.\n",
               ioctl_result);
        retcode = ioctl_result;
        goto finish;
    }

    // Read the first page into memory using command mode and compare it with
    // what we wrote back earlier.
    err = bio_read_block(device, reference_buffer, 0, 1);
    if (err != (ssize_t)device->block_size) {
        printf("Expected to read %zu bytes, actually read %zd. Aborting.\n",
               device->block_size, err);
        retcode = ERR_IO;
        goto finish;
    }

    uint8_t *expected = test_buffer;
    uint8_t *actual = reference_buffer;
    for (uint i = 0; i < device->block_size; i++) {
        if (*actual != *expected) {
            printf("Data mismatch at position %d. Expected %d got %d. "
                   "Aborting.\n", i, *expected, *actual);
            goto finish;
        }
        expected++;
        actual++;
    }

finish:
    free(test_buffer);
    free(reference_buffer);
    return retcode;
}

static int bio_test_device(bdev_t *device) {
    ssize_t num_errors = erase_test(device);
    if (num_errors < 0) {
        printf("error %zd performing erase test\n", num_errors);
        return -1;
    }
    printf("discovered %zd error(s) while testing erase.\n", num_errors);
    if (num_errors) {
        // No point in continuing the tests if we couldn't erase the device.
        printf("not continuing to test writes.\n");
        return -1;
    }

    num_errors = write_test(device);
    printf("Discovered %zd error(s) while testing write.\n", num_errors);
    if (num_errors) {
        return -1;
    }

    printf ("Testing sub-erase...\n");
    bool success = sub_erase_test(device, SUB_ERASE_TEST_SAMPLES);
    if (!success) {
        printf("Discovered errors while testing sub-erase.\n");
        return -1;
    } else {
        printf("No errors while testing sub-erase.\n");
    }

    printf("Testing memory mapped mode...\n");
    status_t test_result = memory_mapped_test(device);
    if (test_result != NO_ERROR) {
        printf("Memory mapped test returned error %d\n", test_result);
    } else {
        printf("Memory mapped mode tests returned successfully\n");
    }

    return 0;
}
