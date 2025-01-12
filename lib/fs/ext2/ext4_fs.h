/*
 *  linux/include/linux/ext4_fs.h
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 *
 *  from
 *
 *  https://ext4.wiki.kernel.org/index.php/Ext4_Disk_Layout
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */
#pragma once

#define RO_COMPAT_HUGE_FILE     0x008
#define RO_COMPAT_DIR_NLINK     0x020
#define RO_COMPAT_EXTRA_ISIZE   0x040
#define RO_COMPAT_METADATA_CSUM 0x400

#define INCOMPAT_64BIT          0x080
#define INCOMPAT_EXTENTS        0x040
#define INCOMPAT_FLEX_BG        0x200

// all fields in little-endian, LE16 and LE32 must be used
// TODO? add a variant of endian_swap_inode
typedef struct {
  uint16_t eh_magic;
  uint16_t eh_entries;
  uint16_t eh_max;
  uint16_t eh_depth;
  uint32_t eh_generation;
} ext4_extent_header;

// all fields in little-endian, LE16 and LE32 must be used
// TODO? add a variant of endian_swap_inode
typedef struct {
  uint32_t ee_block;
  uint16_t ee_len;
  uint16_t ee_start_hi;
  uint32_t ee_start_lo;
} ext4_extent;
