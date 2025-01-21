/*
 * Copyright (c) 2015 Gurjant Kalsi <me@gurjantkalsi.com>
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

#if LK_DEBUGLEVEL > 1

#include <lk/err.h>
#include <math.h>
#include <platform.h>
#include <stdlib.h>
#include <string.h>
#include <lk/console_cmd.h>

#include <lib/bio.h>
#include <lib/fs/spifs.h>

#define FS_NAME "spifs"
#define MNT_PATH "/s"
#define TEST_FILE_PATH "/s/test"
#define TEST_PATH_MAX_SIZE 16

typedef bool(*test_func)(const char *);

typedef struct {
    test_func func;
    const char *name;
    uint32_t toc_pages;
} test;

static bool test_empty_after_format(const char *);
static bool test_double_create_file(const char *);
static bool test_write_read_normal(const char *);
static bool test_write_past_eof(const char *);
static bool test_full_toc(const char *);
static bool test_full_fs(const char *);
static bool test_write_past_end_of_capacity(const char *);
static bool test_rm_reclaim(const char *);
static bool test_corrupt_toc(const char *);
static bool test_write_with_offset(const char *);
static bool test_read_write_big(const char *);
static bool test_rm_active_dirent(const char *);
static bool test_truncate_file(const char *);

static test tests[] = {
    {&test_empty_after_format, "Test no files in ToC after format.", 1},
    {&test_write_read_normal, "Test the normal read/write file paths.", 1},
    {&test_double_create_file, "Test file cannot be created if it already exists.", 1},
    {&test_write_past_eof, "Test that file can grow up to capacity.", 1},
    {&test_full_toc, "Test that files cannot be created once the ToC is full.", 2},
    {&test_full_fs, "Test that files cannot be created once the device is full.", 1},
    {&test_rm_reclaim, "Test that files can be deleted and that used space is reclaimed.", 1},
    {&test_write_past_end_of_capacity, "Test that we cannot write past the capacity of a file.", 1},
    {&test_corrupt_toc, "Test that FS can be mounted with one corrupt ToC.", 1},
    {&test_write_with_offset, "Test that files can be written to at an offset.", 1},
    {&test_read_write_big, "Test that an unaligned ~10kb buffer can be written and read.", 1},
    {&test_rm_active_dirent, "Test that we can remove a file with an open dirent.", 1},
    {&test_truncate_file, "Test that we can truncate a file.", 1},
};

static bool test_setup(const char *dev_name, uint32_t toc_pages) {
    spifs_format_args_t args = {
        .toc_pages = toc_pages,
    };

    status_t res = fs_format_device(FS_NAME, dev_name, (void *)&args);
    if (res != NO_ERROR) {
        printf("spifs_format failed dev = %s, toc_pages = %u, retcode = %d\n",
               dev_name, toc_pages, res);
        return false;
    }

    res = fs_mount(MNT_PATH, FS_NAME, dev_name);
    if (res != NO_ERROR) {
        printf("fs_mount failed path = %s, fs name = %s, dev name = %s,"
               " retcode = %d\n", MNT_PATH, FS_NAME, dev_name, res);
        return false;
    }

    return true;
}

static bool test_teardown(void) {
    if (fs_unmount(MNT_PATH) != NO_ERROR) {
        printf("Unmount failed\n");
        return false;
    }

    return true;;
}

static bool test_empty_after_format(const char *dev_name) {
    dirhandle *dhandle;
    status_t err = fs_open_dir(MNT_PATH, &dhandle);
    if (err != NO_ERROR) {
        return false;
    }

    struct dirent ent;
    if (fs_read_dir(dhandle, &ent) >= 0) {
        fs_close_dir(dhandle);
        return false;
    }

    fs_close_dir(dhandle);
    return true;
}

static bool test_double_create_file(const char *dev_name) {
    status_t status;

    struct dirent *ent = malloc(sizeof(*ent));
    size_t num_files = 0;

    filehandle *handle;
    status = fs_create_file(TEST_FILE_PATH, &handle, 10);
    if (status != NO_ERROR) {
        goto err;
    }
    fs_close_file(handle);

    filehandle *duphandle;
    status = fs_create_file(TEST_FILE_PATH, &duphandle, 20);
    if (status != ERR_ALREADY_EXISTS) {
        goto err;
    }

    dirhandle *dhandle;
    status = fs_open_dir(MNT_PATH, &dhandle);
    if (status != NO_ERROR) {
        goto err;
    }

    while ((status = fs_read_dir(dhandle, ent)) >= 0) {
        num_files++;
    }

    status = NO_ERROR;

    fs_close_dir(dhandle);


err:
    free(ent);

    return status == NO_ERROR ? num_files == 1 : false;
}

static bool test_write_read_normal(const char *dev_name) {
    char test_message[] = "spifs test";
    char test_buf[sizeof(test_message)];

    bdev_t *dev = bio_open(dev_name);
    if (!dev) {
        return false;
    }
    uint8_t erase_byte = dev->erase_byte;
    bio_close(dev);

    filehandle *handle;
    status_t status =
        fs_create_file(TEST_FILE_PATH, &handle, sizeof(test_message));
    if (status != NO_ERROR) {
        return false;
    }

    ssize_t bytes;

    // New files should be initialized to 'erase_byte'
    bytes = fs_read_file(handle, test_buf, 0, sizeof(test_buf));
    if (bytes != sizeof(test_buf)) {
        return false;
    }

    for (size_t i = 0; i < sizeof(test_buf); i++) {
        if (test_buf[i] != erase_byte) {
            return false;
        }
    }

    bytes = fs_write_file(handle, test_message, 0, sizeof(test_message));
    if (bytes != sizeof(test_message)) {
        return false;
    }

    bytes = fs_read_file(handle, test_buf, 0, sizeof(test_buf));
    if (bytes != sizeof(test_buf)) {
        return false;
    }

    status = fs_close_file(handle);
    if (status != NO_ERROR) {
        return false;
    }

    return strncmp(test_message, test_buf, sizeof(test_message)) == 0;
}

static bool test_write_past_eof(const char *dev_name) {
    char test_message[] = "spifs test";

    // Create a 0 length file.
    filehandle *handle;
    status_t status =
        fs_create_file(TEST_FILE_PATH, &handle, 0);
    if (status != NO_ERROR) {
        return false;
    }

    ssize_t bytes = fs_write_file(handle, test_message, 0, sizeof(test_message));
    if (bytes != sizeof(test_message)) {
        return false;
    }

    // Make sure the file grows.
    struct file_stat stat;
    fs_stat_file(handle, &stat);
    if (stat.is_dir != false && stat.size != sizeof(test_message)) {
        return false;
    }

    fs_close_file(handle);

    return true;
}

static bool test_full_toc(const char *dev_name) {
    struct fs_stat stat;

    fs_stat_fs(MNT_PATH, &stat);

    char test_file_name[TEST_PATH_MAX_SIZE];

    filehandle *handle;
    for (size_t i = 0; i < stat.free_inodes; i++) {
        memset(test_file_name, 0, TEST_PATH_MAX_SIZE);

        char filenum[] = "000";
        filenum[0] += i / 100;
        filenum[1] += (i / 10) % 10;
        filenum[2] += i % 10;

        strlcat(test_file_name, MNT_PATH, sizeof(test_file_name));
        strlcat(test_file_name, "/", sizeof(test_file_name));
        strlcat(test_file_name, filenum, sizeof(test_file_name));

        status_t status =
            fs_create_file(test_file_name, &handle, 1);

        if (status != NO_ERROR) {
            return false;
        }

        fs_close_file(handle);
    }

    // There shouldn't be enough space for this file since we've exhausted all
    // the inodes.

    status_t status = fs_create_file(TEST_FILE_PATH, &handle, 1);
    if (status != ERR_TOO_BIG) {
        return false;
    }

    return true;
}

static bool test_rm_reclaim(const char *dev_name) {
    // Create some number of files that's a power of 2;
    size_t n_files = 4;

    struct fs_stat stat;

    fs_stat_fs(MNT_PATH, &stat);

    size_t file_size = stat.free_space / (n_files + 1);

    char test_file_name[TEST_PATH_MAX_SIZE];

    filehandle *handle;
    for (size_t i = 0; i < n_files; i++) {
        memset(test_file_name, 0, TEST_PATH_MAX_SIZE);

        char filenum[] = "000";
        filenum[0] += i / 100;
        filenum[1] += (i / 10) % 10;
        filenum[2] += i % 10;

        strcat(test_file_name, MNT_PATH);
        strcat(test_file_name, filenum);

        status_t status =
            fs_create_file(test_file_name, &handle, file_size);

        if (status != NO_ERROR) {
            return false;
        }

        fs_close_file(handle);
    }

    // Try to create a new Big file.
    char filename[] = "BIGFILE";
    memset(test_file_name, 0, TEST_PATH_MAX_SIZE);
    strcat(test_file_name, MNT_PATH);
    strcat(test_file_name, filename);

    status_t status;

    // This should fail because there's no more space.
    fs_stat_fs(MNT_PATH, &stat);
    status = fs_create_file(test_file_name, &handle, stat.free_space + 1);
    if (status != ERR_TOO_BIG) {
        return false;
    }

    // Delete an existing file to make space for the new file.
    char existing_filename[] = "001";
    memset(test_file_name, 0, TEST_PATH_MAX_SIZE);
    strcat(test_file_name, MNT_PATH);
    strcat(test_file_name, existing_filename);

    status = fs_remove_file(test_file_name);
    if (status != NO_ERROR) {
        return false;
    }


    // Now this should go through because we've reclaimed the space.
    status = fs_create_file(test_file_name, &handle, stat.free_space + 1);
    if (status != NO_ERROR) {
        return false;
    }

    fs_close_file(handle);
    return true;
}

static bool test_full_fs(const char *dev_name) {
    struct fs_stat stat;

    fs_stat_fs(MNT_PATH, &stat);

    char second_file_path[TEST_PATH_MAX_SIZE];
    memset(second_file_path, 0, TEST_PATH_MAX_SIZE);
    strcpy(second_file_path, MNT_PATH);
    strcat(second_file_path, "/fail");

    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, stat.free_space);
    if (status != NO_ERROR) {
        return false;
    }

    fs_close_file(handle);

    // There shouldn't be enough space for this file since we've used all the
    // space.
    status = fs_create_file(second_file_path, &handle, 1);
    if (status != ERR_TOO_BIG) {
        return false;
    }

    return true;
}

static bool test_write_past_end_of_capacity(const char *dev_name) {
    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, 0);
    if (status != NO_ERROR) {
        return false;
    }

    struct file_stat stat;
    status = fs_stat_file(handle, &stat);
    if (status != NO_ERROR) {
        goto finish;
    }

    // We shouldn't be able to write past the capacity of a file.
    char buf[1];
    status = fs_write_file(handle, buf, stat.capacity, 1);
    if (status == ERR_OUT_OF_RANGE) {
        status = NO_ERROR;
    } else {
        status = ERR_IO;
    }

finish:
    fs_close_file(handle);
    return status == NO_ERROR;
}

static bool test_corrupt_toc(const char *dev_name) {
    // Create a zero byte file.
    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, 0);
    if (status != NO_ERROR) {
        return false;
    }

    // Grow the file to one byte. This should trigger a ToC flush. Now the
    // ToC record for this file should exist in both ToCs. Therefore corrupting
    // either of the ToCs will still yield this file readable.
    char buf[1] = { 'a' };
    status = fs_write_file(handle, buf, 0, 1);
    if (status != 1) {
        return false;
    }

    status = fs_close_file(handle);
    if (status != NO_ERROR) {
        return false;
    }

    status = fs_unmount(MNT_PATH);
    if (status != NO_ERROR) {
        return false;
    }

    // Now we're going to manually corrupt one of the ToCs
    bdev_t *dev = bio_open(dev_name);
    if (!dev) {
        return false;
    }

    // Directly write 0s to the block that contains the front-ToC.
    size_t block_size = dev->block_size;
    uint8_t *block_buf = memalign(CACHE_LINE, block_size);
    if (!block_buf) {
        return false;
    }
    memset(block_buf, 0, block_size);

    ssize_t bytes = bio_write_block(dev, block_buf, 0, 1);

    free(block_buf);

    bio_close(dev);

    if (bytes != (ssize_t)block_size) {
        return false;
    }

    // Mount the FS again and make sure that the file we created is still there.
    status = fs_mount(MNT_PATH, FS_NAME, dev_name);
    if (status != NO_ERROR) {
        return false;
    }

    status = fs_open_file(TEST_FILE_PATH, &handle);
    if (status != NO_ERROR) {
        return false;
    }

    struct file_stat stat;
    status = fs_stat_file(handle, &stat);
    if (status != NO_ERROR) {
        return false;
    }

    status = fs_close_file(handle);
    if (status != NO_ERROR) {
        return false;
    }

    return true;
}

static bool test_write_with_offset(const char *dev_name) {
    size_t repeats = 3;
    char test_message[] = "test";
    size_t msg_len = strnlen(test_message, sizeof(test_message));
    char test_buf[msg_len * repeats];

    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, msg_len);
    if (status != NO_ERROR) {
        return false;
    }

    ssize_t bytes;
    for (size_t pos = 0; pos < repeats; pos++) {
        bytes = fs_write_file(handle, test_message, pos * msg_len, msg_len);
        if ((size_t)bytes != msg_len) {
            return false;
        }
    }

    bytes = fs_read_file(handle, test_buf, 0, msg_len * repeats);
    if ((size_t)bytes != msg_len * repeats) {
        return false;
    }

    status = fs_close_file(handle);
    if (status != NO_ERROR) {
        return false;
    }

    bool success = true;
    for (size_t i = 0; i < repeats; i++) {
        success &= (memcmp(test_message,
                           test_buf + i * msg_len,
                           msg_len) == 0);
    }
    return success;
}

static bool test_read_write_big(const char *dev_name) {
    bool success = true;

    size_t buflen = 10013;

    uint8_t *rbuf = malloc(buflen);
    if (!rbuf) {
        return false;
    }

    uint8_t *wbuf = malloc(buflen);
    if (!wbuf) {
        free(rbuf);
        return false;
    }

    for (size_t i = 0; i < buflen; i++) {
        wbuf[i] = rand() % sizeof(uint8_t);
    }

    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, buflen);
    if (status != NO_ERROR) {
        success = false;
        goto err;
    }

    ssize_t bytes = fs_write_file(handle, wbuf, 0, buflen);
    if ((size_t)bytes != buflen) {
        success = false;
        goto err;
    }

    bytes = fs_read_file(handle, rbuf, 0, buflen);
    if ((size_t)bytes != buflen) {
        success = false;
        goto err;
    }

    for (size_t i = 0; i < buflen; i++) {
        if (wbuf[i] != rbuf[i]) {
            success = false;
            break;
        }
    }

err:
    success &= fs_close_file(handle) == NO_ERROR;

    free(rbuf);
    free(wbuf);
    return success;
}

static bool test_rm_active_dirent(const char *dev_name) {
    filehandle *handle;
    status_t status = fs_create_file(TEST_FILE_PATH, &handle, 0);
    if (status != NO_ERROR) {
        return false;
    }

    status = fs_close_file(handle);
    if (status != NO_ERROR) {
        return false;
    }

    dirhandle *dhandle;
    status = fs_open_dir(MNT_PATH, &dhandle);
    if (status != NO_ERROR) {
        return false;
    }

    // Dir handle should now be pointing to the only file in our FS.
    status = fs_remove_file(TEST_FILE_PATH);
    if (status != NO_ERROR) {
        fs_close_dir(dhandle);
        return false;
    }

    bool success = true;
    struct dirent *ent = malloc(sizeof(*ent));
    if (fs_read_dir(dhandle, ent) >= 0) {
        success = false;
    }

    success &= fs_close_dir(dhandle) == NO_ERROR;
    free(ent);

    return success;
}

static bool test_truncate_file(const char *dev_name) {
    filehandle *handle;
    status_t status =
        fs_create_file(TEST_FILE_PATH, &handle, 1024);
    if (status != NO_ERROR) {
        return false;
    }

    // File size is 1024?
    struct file_stat stat;
    fs_stat_file(handle, &stat);
    if (stat.size != 1024) {
        return false;
    }

    status = fs_truncate_file(handle, 512);
    if (status != NO_ERROR) {
        return false;
    }

    fs_stat_file(handle, &stat);
    if (stat.size != 512) {
        return false;
    }

    return fs_close_file(handle) == NO_ERROR;
}

// Run the SPIFS test suite.
static int spifs_test(int argc, const console_cmd_args *argv) {
    if (argc != 3) {
        printf("Expected 3 arguments, got %d.\n", argc);
        return -1;
    }

    // Make sure this block device is legit.
    bdev_t *dev = bio_open(argv[2].str);
    if (!dev) {
        printf("error: could not open block device %s\n", argv[2].str);
        return -1;
    }
    bio_close(dev);

    size_t passed = 0;
    size_t attempted = 0;
    for (size_t i = 0; i < countof(tests); i++) {
        ++attempted;
        if (!test_setup(argv[2].str, tests[i].toc_pages)) {
            printf("Test Setup failed before %s. Exiting.\n", tests[i].name);
            break;
        }

        if (tests[i].func(argv[2].str)) {
            printf(" [Passed] %s\n", tests[i].name);
            ++passed;
        } else {
            printf(" [Failed] %s\n", tests[i].name);
        }

        if (!test_teardown()) {
            printf("Test teardown failed after %s. Exiting.\n", tests[i].name);
            break;
        }
    }
    printf("\nPassed %zu of %zu tests.\n", passed, attempted);

    if (attempted != countof(tests)) {
        printf("(Skipped %zu)\n", countof(tests) - attempted);
    }

    return countof(tests) - passed;
}

// Benchmark SPIFS.
static int spifs_bench(int argc, const console_cmd_args *argv) {
    if (argc != 3) {
        printf("Expected 3 arguments, got %d.\n", argc);
        return -1;
    }

    static const size_t test_buffer_length = 4096;
    static const size_t min_bench_bytes = 0x40;      // 64b
    static const size_t max_bench_bytes = 0x100000;  // 1MiB
    const char *test_file_path = argv[2].str;

    status_t st;
    filehandle *handle;
    int retcode = 0;
    lk_bigtime_t start, end;

    uint8_t *test_buffer = malloc(test_buffer_length);
    memset(test_buffer, 0xAB, test_buffer_length);

    for (size_t file_size = min_bench_bytes;
            file_size <= max_bench_bytes;
            file_size *= 2) {

        printf(" == Benchmark for %zu bytes == \n", file_size);

        // Create file benchmark
        start = current_time_hires();
        st = fs_create_file(test_file_path, &handle, file_size);
        end = current_time_hires();
        printf("\tfs_create_file = %llu usecs\n", end - start);

        if (st != NO_ERROR) {
            printf("SPIFS Benchmark Failed to create a %zu byte file at %s. "
                   "Reason = %d.\n", file_size, test_file_path, st);
            retcode = -1;
            goto finish;
        }


        // Write File Benchmark
        off_t offset = 0;
        ssize_t n_bytes = 0;
        start = current_time_hires();
        do {
            size_t write_length = MIN(
                                      test_buffer_length,
                                      file_size - offset
                                  );
            n_bytes = fs_write_file(handle, test_buffer, offset, write_length);
            offset += n_bytes;
        } while (n_bytes > 0);

        end = current_time_hires();
        printf("\tfs_write_file = %llu usecs\n", end - start);

        if (n_bytes < 0) {
            printf("SPIFS Benchmark Failed to write to file at %s. "
                   "Reason = %zd.\n", test_file_path, n_bytes);
            retcode = -1;
            fs_close_file(handle);
            goto finish;
        }

        // Read File Benchmark
        offset = 0;
        n_bytes = 0;
        start = current_time_hires();
        do {
            n_bytes = fs_read_file(handle, test_buffer, offset,
                                   test_buffer_length);
            offset += n_bytes;
        } while (n_bytes > 0);

        end = current_time_hires();
        printf("\tfs_read_file = %llu usecs\n", end - start);

        if (n_bytes < 0) {
            printf("SPIFS Benchmark Failed to read from file at %s. "
                   "Reason = %zd.\n", test_file_path, n_bytes);
            retcode = -1;
            fs_close_file(handle);
            goto finish;
        }

        st = fs_close_file(handle);
        if (st != NO_ERROR) {
            printf("SPIFS Benchmark Failed to close file at %s. "
                   "Reason = %d.\n", test_file_path, st);
            retcode = -1;
            goto finish;
        }

        start = current_time_hires();
        st = fs_remove_file(test_file_path);
        end = current_time_hires();
        printf("\tfs_remove_file = %llu usecs\n", end - start);

        if (st != NO_ERROR) {
            printf("SPIFS Benchmark Failed to remove file at %s. "
                   "Reason = %d.\n", test_file_path, st);
            retcode = -1;
            goto finish;
        }

        printf("\n");
    }
finish:
    free(test_buffer);
    return retcode;
}

static int cmd_spifs(int argc, const console_cmd_args *argv) {
    if (argc < 3) {
        printf("not enough arguments:\n");
usage:
        printf("%s test <device>\n", argv[0].str);
        printf("%s bench <path>\n", argv[0].str);
        return -1;
    }

    if (!strcmp(argv[1].str, "test")) {
        return spifs_test(argc, argv);
    } else if (!strcmp(argv[1].str, "bench")) {
        return spifs_bench(argc, argv);
    }

    // Command not found.
    goto usage;
}

STATIC_COMMAND_START
STATIC_COMMAND("spifs", "commands related to the spifs implementation.", &cmd_spifs)
STATIC_COMMAND_END(spifs);

#endif  // LK_DEBUGLEVEL > 1
