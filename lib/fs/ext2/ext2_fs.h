/*
 *  linux/include/linux/ext2_fs.h
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 *
 *  from
 *
 *  linux/include/linux/minix_fs.h
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

#ifndef _LINUX_EXT2_FS_H
#define _LINUX_EXT2_FS_H

#include <sys/types.h>
#include <stdint.h>

//#include <linux/types.h>
//#include <linux/magic.h>

/*
 * The second extended filesystem constants/structures
 */

/*
 * Special inode numbers
 */
#define EXT2_BAD_INO         1  /* Bad blocks inode */
#define EXT2_ROOT_INO        2  /* Root inode */
#define EXT2_BOOT_LOADER_INO     5  /* Boot loader inode */
#define EXT2_UNDEL_DIR_INO   6  /* Undelete directory inode */

/* First non-reserved inode for old ext2 filesystems */
#define EXT2_GOOD_OLD_FIRST_INO 11

/*
 * Maximal count of links to a file
 */
#define EXT2_LINK_MAX       32000

/*
 * Macro-instructions used to manage several block sizes
 */
#define EXT2_MIN_BLOCK_SIZE     1024
#define EXT2_MAX_BLOCK_SIZE     4096
#define EXT2_MIN_BLOCK_LOG_SIZE       10
#define EXT2_BLOCK_SIZE(s)      ((uint32_t)EXT2_MIN_BLOCK_SIZE << (s).s_log_block_size)
#define EXT2_ADDR_PER_BLOCK(s)      (EXT2_BLOCK_SIZE(s) / sizeof (uint32_t))
#define EXT2_BLOCK_SIZE_BITS(s) ((s).s_log_block_size + 10)
#define EXT2_INODE_SIZE(s)  (((s).s_rev_level == EXT2_GOOD_OLD_REV) ? \
                 EXT2_GOOD_OLD_INODE_SIZE : \
                 (s).s_inode_size)
#define EXT2_FIRST_INO(s)   (((s).s_rev_level == EXT2_GOOD_OLD_REV) ? \
                 EXT2_GOOD_OLD_FIRST_INO : \
                 (s).s_first_ino)

/*
 * Macro-instructions used to manage fragments
 */
#define EXT2_MIN_FRAG_SIZE      1024
#define EXT2_MAX_FRAG_SIZE      4096
#define EXT2_MIN_FRAG_LOG_SIZE        10
#define EXT2_FRAG_SIZE(s)       (EXT2_MIN_FRAG_SIZE << (s).s_log_frag_size)
#define EXT2_FRAGS_PER_BLOCK(s) (EXT2_BLOCK_SIZE(s) / EXT2_FRAG_SIZE(s))

/*
 * Structure of a blocks group descriptor
 * for ext4 compat, each descriptor is now 64 bytes in size
 * when mounting ext2, bio_read will only populate the lower 32 bytes of each descriptor
 */
struct ext2_group_desc {
    uint32_t    bg_block_bitmap;        /* Blocks bitmap block */
    uint32_t    bg_inode_bitmap;        /* Inodes bitmap block */
    uint32_t    bg_inode_table;     /* Inodes table block */
    uint16_t    bg_free_blocks_count;   /* Free blocks count */
    uint16_t    bg_free_inodes_count;   /* Free inodes count */
    uint16_t    bg_used_dirs_count; /* Directories count */
    uint16_t    bg_pad;
    uint32_t    bg_reserved[3];
    uint8_t     bg_reserved2[32]; // TODO, fill in the other fields
};

/*
 * Macro-instructions used to manage group descriptors
 */
#define EXT2_BLOCKS_PER_GROUP(s)    ((s).s_blocks_per_group)
#define EXT2_DESC_PER_BLOCK(s)      (EXT2_BLOCK_SIZE(s) / sizeof (struct ext2_group_desc))
#define EXT2_INODES_PER_GROUP(s)    ((s).s_inodes_per_group)

/*
 * Constants relative to the data blocks
 */
#define EXT2_NDIR_BLOCKS        12
#define EXT2_IND_BLOCK          EXT2_NDIR_BLOCKS
#define EXT2_DIND_BLOCK         (EXT2_IND_BLOCK + 1)
#define EXT2_TIND_BLOCK         (EXT2_DIND_BLOCK + 1)
#define EXT2_N_BLOCKS           (EXT2_TIND_BLOCK + 1)

/*
 * Structure of an inode on the disk
 * endian_swap_inode and ext2_load_inode deal with byte-order
 */
struct ext2_inode {
    uint16_t    i_mode;     /* File mode */
    uint16_t    i_uid;      /* Low 16 bits of Owner Uid */
    uint32_t    i_size;     /* Size in bytes */
    uint32_t    i_atime;    /* Access time */
    uint32_t    i_ctime;    /* Creation time */
    uint32_t    i_mtime;    /* Modification time */
    uint32_t    i_dtime;    /* Deletion Time */
    uint16_t    i_gid;      /* Low 16 bits of Group Id */
    uint16_t    i_links_count;  /* Links count */
    uint32_t    i_blocks;   /* Blocks count */
    uint32_t    i_flags;    /* File flags */
    union {
        struct {
            uint32_t  l_i_reserved1;
        } linux1;
        struct {
            uint32_t  h_i_translator;
        } hurd1;
        struct {
            uint32_t  m_i_reserved1;
        } masix1;
    } osd1;             /* OS dependent 1 */
    uint32_t    i_block[EXT2_N_BLOCKS];/* Pointers to blocks */
    uint32_t    i_generation;   /* File version (for NFS) */
    uint32_t    i_file_acl; /* File ACL */
    uint32_t    i_dir_acl;  /* Directory ACL */
    uint32_t    i_faddr;    /* Fragment address */
    union {
        struct {
            uint8_t l_i_frag;   /* Fragment number */
            uint8_t l_i_fsize;  /* Fragment size */
            uint16_t    i_pad1;
            uint16_t    l_i_uid_high;   /* these 2 fields    */
            uint16_t    l_i_gid_high;   /* were reserved2[0] */
            uint32_t    l_i_reserved2;
        } linux2;
        struct {
            uint8_t h_i_frag;   /* Fragment number */
            uint8_t h_i_fsize;  /* Fragment size */
            uint16_t    h_i_mode_high;
            uint16_t    h_i_uid_high;
            uint16_t    h_i_gid_high;
            uint32_t    h_i_author;
        } hurd2;
        struct {
            uint8_t m_i_frag;   /* Fragment number */
            uint8_t m_i_fsize;  /* Fragment size */
            uint16_t    m_pad1;
            uint32_t    m_i_reserved2[2];
        } masix2;
    } osd2;             /* OS dependent 2 */
};

#define i_size_high i_dir_acl

#define i_reserved1 osd1.linux1.l_i_reserved1
#define i_frag      osd2.linux2.l_i_frag
#define i_fsize     osd2.linux2.l_i_fsize
#define i_uid_low   i_uid
#define i_gid_low   i_gid
#define i_uid_high  osd2.linux2.l_i_uid_high
#define i_gid_high  osd2.linux2.l_i_gid_high
#define i_reserved2 osd2.linux2.l_i_reserved2

/*
 * File system states
 */
#define EXT2_VALID_FS           0x0001  /* Unmounted cleanly */
#define EXT2_ERROR_FS           0x0002  /* Errors detected */

/*
 * Mount flags
 */
#define EXT2_MOUNT_CHECK        0x000001  /* Do mount-time checks */
#define EXT2_MOUNT_OLDALLOC     0x000002  /* Don't use the new Orlov allocator */
#define EXT2_MOUNT_GRPID        0x000004  /* Create files with directory's group */
#define EXT2_MOUNT_DEBUG        0x000008  /* Some debugging messages */
#define EXT2_MOUNT_ERRORS_CONT      0x000010  /* Continue on errors */
#define EXT2_MOUNT_ERRORS_RO        0x000020  /* Remount fs ro on errors */
#define EXT2_MOUNT_ERRORS_PANIC     0x000040  /* Panic on errors */
#define EXT2_MOUNT_MINIX_DF     0x000080  /* Mimics the Minix statfs */
#define EXT2_MOUNT_NOBH         0x000100  /* No buffer_heads */
#define EXT2_MOUNT_NO_UID32     0x000200  /* Disable 32-bit UIDs */
#define EXT2_MOUNT_XATTR_USER       0x004000  /* Extended user attributes */
#define EXT2_MOUNT_POSIX_ACL        0x008000  /* POSIX Access Control Lists */
#define EXT2_MOUNT_XIP          0x010000  /* Execute in place */
#define EXT2_MOUNT_USRQUOTA     0x020000 /* user quota */
#define EXT2_MOUNT_GRPQUOTA     0x040000 /* group quota */


#define clear_opt(o, opt)       o &= ~EXT2_MOUNT_##opt
#define set_opt(o, opt)         o |= EXT2_MOUNT_##opt
#define test_opt(sb, opt)       (EXT2_SB(sb)->s_mount_opt & \
                     EXT2_MOUNT_##opt)
/*
 * Maximal mount counts between two filesystem checks
 */
#define EXT2_DFL_MAX_MNT_COUNT      20  /* Allow 20 mounts */
#define EXT2_DFL_CHECKINTERVAL      0   /* Don't use interval check */

/*
 * Behaviour when detecting errors
 */
#define EXT2_ERRORS_CONTINUE        1   /* Continue execution */
#define EXT2_ERRORS_RO          2   /* Remount fs read-only */
#define EXT2_ERRORS_PANIC       3   /* Panic */
#define EXT2_ERRORS_DEFAULT     EXT2_ERRORS_CONTINUE

#define EXT2_SUPER_MAGIC    0xEF53
#define EXT3_SUPER_MAGIC    0xEF53
#define EXT4_SUPER_MAGIC    0xEF53

/*
 * Structure of the super block
 */
struct ext2_super_block {
    uint32_t    s_inodes_count;     /* Inodes count */
    uint32_t    s_blocks_count;     /* Blocks count */
    uint32_t    s_r_blocks_count;   /* Reserved blocks count */
    uint32_t    s_free_blocks_count;    /* Free blocks count */
    uint32_t    s_free_inodes_count;    /* Free inodes count */
    uint32_t    s_first_data_block; /* First Data Block */
    uint32_t    s_log_block_size;   /* Block size */
    uint32_t    s_log_frag_size;    /* Fragment size */
    uint32_t    s_blocks_per_group; /* # Blocks per group */
    uint32_t    s_frags_per_group;  /* # Fragments per group */
    uint32_t    s_inodes_per_group; /* # Inodes per group */
    uint32_t    s_mtime;        /* Mount time */
    uint32_t    s_wtime;        /* Write time */
    uint16_t    s_mnt_count;        /* Mount count */
    uint16_t    s_max_mnt_count;    /* Maximal mount count */
    uint16_t    s_magic;        /* Magic signature */
    uint16_t    s_state;        /* File system state */
    uint16_t    s_errors;       /* Behaviour when detecting errors */
    uint16_t    s_minor_rev_level;  /* minor revision level */
    uint32_t    s_lastcheck;        /* time of last check */
    uint32_t    s_checkinterval;    /* max. time between checks */
    uint32_t    s_creator_os;       /* OS */
    uint32_t    s_rev_level;        /* Revision level */
    uint16_t    s_def_resuid;       /* Default uid for reserved blocks */
    uint16_t    s_def_resgid;       /* Default gid for reserved blocks */
    /*
     * These fields are for EXT2_DYNAMIC_REV superblocks only.
     *
     * Note: the difference between the compatible feature set and
     * the incompatible feature set is that if there is a bit set
     * in the incompatible feature set that the kernel doesn't
     * know about, it should refuse to mount the filesystem.
     *
     * e2fsck's requirements are more strict; if it doesn't know
     * about a feature in either the compatible or incompatible
     * feature set, it must abort and not try to meddle with
     * things it doesn't understand...
     */
    uint32_t    s_first_ino;        /* First non-reserved inode */
    uint16_t   s_inode_size;        /* size of inode structure */
    uint16_t    s_block_group_nr;   /* block group # of this superblock */
    uint32_t    s_feature_compat;   /* compatible feature set */
    uint32_t    s_feature_incompat;     /* incompatible feature set */
    uint32_t    s_feature_ro_compat;    /* readonly-compatible feature set */
    uint8_t s_uuid[16];     /* 128-bit uuid for volume */
    char    s_volume_name[16];  /* volume name */
    char    s_last_mounted[64];     /* directory where last mounted */
    uint32_t    s_algorithm_usage_bitmap; /* For compression */
    /*
     * Performance hints.  Directory preallocation should only
     * happen if the EXT2_COMPAT_PREALLOC flag is on.
     */
    uint8_t s_prealloc_blocks;  /* Nr of blocks to try to preallocate*/
    uint8_t s_prealloc_dir_blocks;  /* Nr to preallocate for dirs */
    uint16_t    s_padding1;
    /*
     * Journaling support valid if EXT3_FEATURE_COMPAT_HAS_JOURNAL set.
     */
    uint8_t s_journal_uuid[16]; /* uuid of journal superblock */
    uint32_t    s_journal_inum;     /* inode number of journal file */
    uint32_t    s_journal_dev;      /* device number of journal file */
    uint32_t    s_last_orphan;      /* start of list of inodes to delete */
    uint32_t    s_hash_seed[4];     /* HTREE hash seed */
    uint8_t     s_def_hash_version; /* Default hash version to use */
    uint8_t     s_jnl_backup_type;
    uint16_t    s_desc_size;        // size of group descriptors, if INCOMPAT_64BIT is set
    uint32_t    s_default_mount_opts;
    uint32_t    s_first_meta_bg;    /* First metablock block group */
    // the following are added in at least ext4
    uint32_t    s_mkfs_time;
    uint32_t    s_jnl_blocks[17];
    // these upper 32bits are only used if EXT4_FEATURE_COMPAT_64BIT is set
    uint32_t    s_blocks_count_hi;
    uint32_t    s_r_blocks_count_hi;
    uint32_t    s_free_blocks_count_hi;
    uint16_t    s_min_extra_isize;
    uint16_t    s_want_extra_isize;
    uint32_t    s_flags;
    uint16_t    s_raid_stride;
    uint16_t    s_mmp_interval;
    uint64_t    s_mmp_block;
    uint32_t    s_raid_stripe_width;
    uint8_t     s_log_groups_per_flex;
    uint8_t     s_checksum_type;
    uint16_t    s_reserved_pad;
    uint64_t    s_kbytes_written;
    uint32_t    s_snapshot_inum;
    uint32_t    s_snapshot_id;
    uint64_t    s_snapshot_r_blocks_count;
    uint32_t    s_snapshot_list;
    uint32_t    s_error_count;

    uint32_t    s_first_error_time;
    uint32_t    s_first_error_ino;
    uint64_t    s_first_error_block;
    uint8_t     s_first_error_func[32];
    uint32_t    s_first_error_line;

    uint32_t    s_last_error_time;
    uint32_t    s_last_error_ino;
    uint32_t    s_last_error_line;
    uint64_t    s_last_error_block;
    uint8_t     s_last_error_func[32];

    uint8_t     s_mount_opts[64];
    uint32_t    s_usr_quota_inum;
    uint32_t    s_grp_quota_inum;
    uint32_t    s_overhead_blocks;
    uint32_t    s_backup_bgs[2];
    uint8_t     s_encrypt_algos[4];
    uint8_t     s_encrypt_pw_salt[16];
    uint32_t    s_lpf_ino;
    uint32_t    s_prj_quota_inum;
    uint32_t    s_checksum_seed;
    uint8_t     s_wtime_hi;
    uint8_t     s_mtime_hi;
    uint8_t     s_mkfs_time_hi;
    uint8_t     s_lastcheck_hi;
    uint8_t     s_first_error_time_hi;
    uint8_t     s_last_error_time_hi;
    uint8_t     s_pad[2];
    uint16_t    s_encoding;
    uint16_t    s_encoding_flags;
    uint32_t    s_reserved[95];     /* Padding to the end of the block */
    uint32_t    s_checksum;
};

/*
 * Codes for operating systems
 */
#define EXT2_OS_LINUX       0
#define EXT2_OS_HURD        1
#define EXT2_OS_MASIX       2
#define EXT2_OS_FREEBSD     3
#define EXT2_OS_LITES       4

/*
 * Revision levels
 */
#define EXT2_GOOD_OLD_REV   0   /* The good old (original) format */
#define EXT2_DYNAMIC_REV    1   /* V2 format w/ dynamic inode sizes */

#define EXT2_CURRENT_REV    EXT2_GOOD_OLD_REV
#define EXT2_MAX_SUPP_REV   EXT2_DYNAMIC_REV

#define EXT2_GOOD_OLD_INODE_SIZE 128

/*
 * Feature set definitions
 */

#define EXT2_HAS_COMPAT_FEATURE(sb,mask)            \
    ( (sb).s_feature_compat & cpu_to_le32(mask) )
#define EXT2_HAS_RO_COMPAT_FEATURE(sb,mask)         \
    ( (sb).s_feature_ro_compat & cpu_to_le32(mask) )
#define EXT2_HAS_INCOMPAT_FEATURE(sb,mask)          \
    ( (sb).s_feature_incompat & cpu_to_le32(mask) )
#define EXT2_SET_COMPAT_FEATURE(sb,mask)            \
    (sb).s_feature_compat |= cpu_to_le32(mask)
#define EXT2_SET_RO_COMPAT_FEATURE(sb,mask)         \
    (sb).s_feature_ro_compat |= cpu_to_le32(mask)
#define EXT2_SET_INCOMPAT_FEATURE(sb,mask)          \
    (sb).s_feature_incompat |= cpu_to_le32(mask)
#define EXT2_CLEAR_COMPAT_FEATURE(sb,mask)          \
    (sb).s_feature_compat &= ~cpu_to_le32(mask)
#define EXT2_CLEAR_RO_COMPAT_FEATURE(sb,mask)           \
    (sb).s_feature_ro_compat &= ~cpu_to_le32(mask)
#define EXT2_CLEAR_INCOMPAT_FEATURE(sb,mask)            \
    (sb).s_feature_incompat &= ~cpu_to_le32(mask)

#define EXT2_FEATURE_COMPAT_DIR_PREALLOC    0x0001
#define EXT2_FEATURE_COMPAT_IMAGIC_INODES   0x0002
#define EXT3_FEATURE_COMPAT_HAS_JOURNAL     0x0004
#define EXT2_FEATURE_COMPAT_EXT_ATTR        0x0008
#define EXT2_FEATURE_COMPAT_RESIZE_INO      0x0010
#define EXT2_FEATURE_COMPAT_DIR_INDEX       0x0020
#define EXT2_FEATURE_COMPAT_ANY         0xffffffff

#define EXT2_FEATURE_RO_COMPAT_SPARSE_SUPER 0x0001
#define EXT2_FEATURE_RO_COMPAT_LARGE_FILE   0x0002
#define EXT2_FEATURE_RO_COMPAT_BTREE_DIR    0x0004
#define EXT2_FEATURE_RO_COMPAT_ANY      0xffffffff

#define EXT2_FEATURE_INCOMPAT_COMPRESSION   0x0001
#define EXT2_FEATURE_INCOMPAT_FILETYPE      0x0002
#define EXT3_FEATURE_INCOMPAT_RECOVER       0x0004
#define EXT3_FEATURE_INCOMPAT_JOURNAL_DEV   0x0008
#define EXT2_FEATURE_INCOMPAT_META_BG       0x0010
#define EXT2_FEATURE_INCOMPAT_ANY       0xffffffff

#define EXT2_FEATURE_COMPAT_SUPP    EXT2_FEATURE_COMPAT_EXT_ATTR
#define EXT2_FEATURE_INCOMPAT_SUPP  (EXT2_FEATURE_INCOMPAT_FILETYPE| \
                     EXT2_FEATURE_INCOMPAT_META_BG)
#define EXT2_FEATURE_RO_COMPAT_SUPP (EXT2_FEATURE_RO_COMPAT_SPARSE_SUPER| \
                     EXT2_FEATURE_RO_COMPAT_LARGE_FILE| \
                     EXT2_FEATURE_RO_COMPAT_BTREE_DIR)
#define EXT2_FEATURE_RO_COMPAT_UNSUPPORTED  ~EXT2_FEATURE_RO_COMPAT_SUPP
#define EXT2_FEATURE_INCOMPAT_UNSUPPORTED   ~EXT2_FEATURE_INCOMPAT_SUPP

/*
 * Default values for user and/or group using reserved blocks
 */
#define EXT2_DEF_RESUID     0
#define EXT2_DEF_RESGID     0

/*
 * Default mount options
 */
#define EXT2_DEFM_DEBUG     0x0001
#define EXT2_DEFM_BSDGROUPS 0x0002
#define EXT2_DEFM_XATTR_USER    0x0004
#define EXT2_DEFM_ACL       0x0008
#define EXT2_DEFM_UID16     0x0010
/* Not used by ext2, but reserved for use by ext3 */
#define EXT3_DEFM_JMODE     0x0060
#define EXT3_DEFM_JMODE_DATA    0x0020
#define EXT3_DEFM_JMODE_ORDERED 0x0040
#define EXT3_DEFM_JMODE_WBACK   0x0060

/*
 * Structure of a directory entry
 */
#define EXT2_NAME_LEN 255

struct ext2_dir_entry {
    uint32_t    inode;          /* Inode number */
    uint16_t    rec_len;        /* Directory entry length */
    uint16_t    name_len;       /* Name length */
    char    name[EXT2_NAME_LEN];    /* File name */
};

/*
 * The new version of the directory entry.  Since EXT2 structures are
 * stored in intel byte order, and the name_len field could never be
 * bigger than 255 chars, it's safe to reclaim the extra byte for the
 * file_type field.
 */
struct ext2_dir_entry_2 {
    uint32_t    inode;          /* Inode number */
    uint16_t    rec_len;        /* Directory entry length */
    uint8_t name_len;       /* Name length */
    uint8_t file_type;
    char    name[EXT2_NAME_LEN];    /* File name */
};

/*
 * Ext2 directory file types.  Only the low 3 bits are used.  The
 * other bits are reserved for now.
 */
enum {
    EXT2_FT_UNKNOWN,
    EXT2_FT_REG_FILE,
    EXT2_FT_DIR,
    EXT2_FT_CHRDEV,
    EXT2_FT_BLKDEV,
    EXT2_FT_FIFO,
    EXT2_FT_SOCK,
    EXT2_FT_SYMLINK,
    EXT2_FT_MAX
};

/*
 * EXT2_DIR_PAD defines the directory entries boundaries
 *
 * NOTE: It must be a multiple of 4
 */
#define EXT2_DIR_PAD            4
#define EXT2_DIR_ROUND          (EXT2_DIR_PAD - 1)
#define EXT2_DIR_REC_LEN(name_len)  (((name_len) + 8 + EXT2_DIR_ROUND) & \
                     ~EXT2_DIR_ROUND)

#endif  /* _LINUX_EXT2_FS_H */
