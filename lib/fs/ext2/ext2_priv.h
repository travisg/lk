/*
 * Copyright (c) 2007-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lib/bio.h>
#include <lib/bcache.h>
#include <lib/fs.h>
#include "ext2_fs.h"
#include "ext4_fs.h"

typedef uint32_t blocknum_t;
typedef uint32_t inodenum_t;
typedef uint32_t groupnum_t;

typedef struct {
    bdev_t *dev;
    bcache_t cache;

    struct ext2_super_block sb;
    int s_group_count;
    struct ext2_group_desc *gd;
    struct ext2_inode root_inode;

    bool has_new_directory_entries;
} ext2_t;

struct cache_block {
    blocknum_t num;
    void *ptr;
};

/* open file handle */
typedef struct {
    ext2_t *ext2;

    struct cache_block ind_cache[3]; // cache of indirect blocks as they're scanned
    struct ext2_inode inode;
} ext2_file_t;

/* internal routines */
int ext2_load_inode(ext2_t *ext2, inodenum_t num, struct ext2_inode *inode);
int ext2_lookup(ext2_t *ext2, const char *path, inodenum_t *inum); // path to inode

/* io */
int ext2_read_block(ext2_t *ext2, void *buf, blocknum_t bnum);
int ext2_get_block(ext2_t *ext2, void **ptr, blocknum_t bnum);
int ext2_put_block(ext2_t *ext2, blocknum_t bnum);

off_t ext2_file_len(ext2_t *ext2, struct ext2_inode *inode);
ssize_t ext2_read_inode(ext2_t *ext2, struct ext2_inode *inode, void *buf, off_t offset, size_t len);
int ext2_read_link(ext2_t *ext2, struct ext2_inode *inode, char *str, size_t len);

/* fs api */
status_t ext2_mount(bdev_t *dev, fscookie **cookie);
status_t ext2_unmount(fscookie *cookie);
status_t ext2_open_file(fscookie *cookie, const char *path, filecookie **fcookie);
ssize_t ext2_read_file(filecookie *fcookie, void *buf, off_t offset, size_t len);
status_t ext2_close_file(filecookie *fcookie);
status_t ext2_stat_file(filecookie *fcookie, struct file_stat *);

status_t ext2_opendir(fscookie *, const char *, dircookie **);
status_t ext2_readdir(dircookie *, struct dirent *);
status_t ext2_closedir(dircookie *);

/* mode stuff */
#define S_IFMT      0170000
#define S_IFIFO     0010000
#define S_IFCHR     0020000
#define S_IFDIR     0040000
#define S_IFBLK     0060000
#define S_IFREG     0100000
#define S_IFLNK     0120000
#define S_IFSOCK    0140000

#define S_ISFIFO(mode) (((mode) & S_IFMT) == S_IFIFO)
#define S_ISCHR(mode) (((mode) & S_IFMT) == S_IFCHR)
#define S_ISDIR(mode) (((mode) & S_IFMT) == S_IFDIR)
#define S_ISBLK(mode) (((mode) & S_IFMT) == S_IFBLK)
#define S_ISREG(mode) (((mode) & S_IFMT) == S_IFREG)
#define S_ISLNK(mode) (((mode) & S_IFMT) == S_IFLNK)
#define S_ISSOCK(mode) (((mode) & S_IFMT) == S_IFSOCK)

