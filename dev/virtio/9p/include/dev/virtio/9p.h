/*
 * Copyright (c) 2023, Google Inc. All rights reserved.
 * Author: codycswong@google.com (Cody Wong)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <dev/virtio.h>
#include <lib/bio.h>
#include <lk/compiler.h>
#include <kernel/spinlock.h>
#include <sys/types.h>

#define VIRTIO_9P_RING_IDX 0
#define VIRTIO_9P_RING_SIZE 128

// Assuming there can be only one outstanding request. Pick a 16-bit unsigned
// value arbtrarily as the default tag for no good reason.
#define P9_TAG_DEFAULT ((uint16_t)0x15)
#define P9_TAG_NOTAG ((uint16_t)~0)

#define P9_FID_NOFID ((uint32_t)~0)

#define P9_UNAME_NONUNAME ((uint32_t)~0)

#define P9_MAXWELEM 16

#ifdef V9P_HOST_DIR
#define V9P_MOUNT_ANAME V9P_HOST_DIR
#else
#define V9P_MOUNT_ANAME ""
#endif // V9P_MOUNT_ANAME

// Linux file flags
#define O_RDONLY    00000000
#define O_WRONLY    00000001
#define O_RDWR      00000002
#ifndef O_CREAT
#define O_CREAT     00000100
#endif
#ifndef O_EXCL
#define O_EXCL      00000200
#endif
#ifndef O_NOCTTY
#define O_NOCTTY    00000400
#endif
#ifndef O_TRUNC
#define O_TRUNC     00001000
#endif
#ifndef O_APPEND
#define O_APPEND    00002000
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK  00004000
#endif
#ifndef O_DSYNC
#define O_DSYNC     00010000
#endif
#ifndef FASYNC
#define FASYNC      00020000
#endif
#ifndef O_DIRECT
#define O_DIRECT    00040000
#endif
#ifndef O_LARGEFILE
#define O_LARGEFILE 00100000
#endif
#ifndef O_DIRECTORY
#define O_DIRECTORY 00200000
#endif
#ifndef O_NOFOLLOW
#define O_NOFOLLOW  00400000
#endif
#ifndef O_NOATIME
#define O_NOATIME   01000000
#endif
#ifndef O_CLOEXEC
#define O_CLOEXEC   02000000
#endif

// POSIX file modes
#define S_IRWXU  00700 // user (file owner) has read, write, and execute permission
#define S_IRUSR  00400 // user has read permission
#define S_IWUSR  00200 // user has write permission
#define S_IXUSR  00100 // user has execute permission
#define S_IRWXG  00070 // group has read, write, and execute permission
#define S_IRGRP  00040 // group has read permission
#define S_IWGRP  00020 // group has write permission
#define S_IXGRP  00010 // group has execute permission
#define S_IRWXO  00007 // others have read, write, and execute permission
#define S_IROTH  00004 // others have read permission
#define S_IWOTH  00002 // others have write permission
#define S_IXOTH  00001 // others have execute permission
/* According to POSIX, the effect when other bits are set in mode is
 * unspecified.  On Linux, the following bits are also honored in mode: */
#define S_ISUID  0004000 // set-user-ID bit
#define S_ISGID  0002000 // set-group-ID bit (see inode(7)).
#define S_ISVTX  0001000 // sticky bit (see inode(7)).

// Linux file modes
#define S_IFMT  00170000
#define S_IFSOCK 0140000
#define S_IFLNK  0120000
#define S_IFREG  0100000
#define S_IFBLK  0060000
#define S_IFDIR  0040000
#define S_IFCHR  0020000
#define S_IFIFO  0010000
#define S_ISUID  0004000
#define S_ISGID  0002000
#define S_ISVTX  0001000

#define S_ISLNK(m)  (((m) & S_IFMT) == S_IFLNK)
#define S_ISREG(m)  (((m) & S_IFMT) == S_IFREG)
#define S_ISDIR(m)  (((m) & S_IFMT) == S_IFDIR)
#define S_ISCHR(m)  (((m) & S_IFMT) == S_IFCHR)
#define S_ISBLK(m)  (((m) & S_IFMT) == S_IFBLK)
#define S_ISFIFO(m) (((m) & S_IFMT) == S_IFIFO)
#define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)

#define P9_GETATTR_MODE         0x00000001ULL
#define P9_GETATTR_NLINK        0x00000002ULL
#define P9_GETATTR_UID          0x00000004ULL
#define P9_GETATTR_GID          0x00000008ULL
#define P9_GETATTR_RDEV         0x00000010ULL
#define P9_GETATTR_ATIME        0x00000020ULL
#define P9_GETATTR_MTIME        0x00000040ULL
#define P9_GETATTR_CTIME        0x00000080ULL
#define P9_GETATTR_INO          0x00000100ULL
#define P9_GETATTR_SIZE         0x00000200ULL
#define P9_GETATTR_BLOCKS       0x00000400ULL

#define P9_GETATTR_BTIME        0x00000800ULL
#define P9_GETATTR_GEN          0x00001000ULL
#define P9_GETATTR_DATA_VERSION 0x00002000ULL

#define P9_GETATTR_BASIC        0x000007ffULL /* Mask for fields up to BLOCKS */
#define P9_GETATTR_ALL          0x00003fffULL /* Mask for All fields above */

enum virtio_9p_msg_type_t {
    P9_TLERROR = 6,
    P9_RLERROR,
    P9_TSTATFS = 8,
    P9_RSTATFS,
    P9_TLOPEN = 12,
    P9_RLOPEN,
    P9_TLCREATE = 14,
    P9_RLCREATE,
    P9_TSYMLINK = 16,
    P9_RSYMLINK,
    P9_TMKNOD = 18,
    P9_RMKNOD,
    P9_TRENAME = 20,
    P9_RRENAME,
    P9_TREADLINK = 22,
    P9_RREADLINK,
    P9_TGETATTR = 24,
    P9_RGETATTR,
    P9_TSETATTR = 26,
    P9_RSETATTR,
    P9_TXATTRWALK = 30,
    P9_RXATTRWALK,
    P9_TXATTRCREATE = 32,
    P9_RXATTRCREATE,
    P9_TREADDIR = 40,
    P9_RREADDIR,
    P9_TFSYNC = 50,
    P9_RFSYNC,
    P9_TLOCK = 52,
    P9_RLOCK,
    P9_TGETLOCK = 54,
    P9_RGETLOCK,
    P9_TLINK = 70,
    P9_RLINK,
    P9_TMKDIR = 72,
    P9_RMKDIR,
    P9_TRENAMEAT = 74,
    P9_RRENAMEAT,
    P9_TUNLINKAT = 76,
    P9_RUNLINKAT,
    P9_TVERSION = 100,
    P9_RVERSION,
    P9_TAUTH = 102,
    P9_RAUTH,
    P9_TATTACH = 104,
    P9_RATTACH,
    P9_TERROR = 106,
    P9_RERROR,
    P9_TFLUSH = 108,
    P9_RFLUSH,
    P9_TWALK = 110,
    P9_RWALK,
    P9_TOPEN = 112,
    P9_ROPEN,
    P9_TCREATE = 114,
    P9_RCREATE,
    P9_TREAD = 116,
    P9_RREAD,
    P9_TWRITE = 118,
    P9_RWRITE,
    P9_TCLUNK = 120,
    P9_RCLUNK,
    P9_TREMOVE = 122,
    P9_RREMOVE,
    P9_TSTAT = 124,
    P9_RSTAT,
    P9_TWSTAT = 126,
    P9_RWSTAT,
};

typedef struct _virtio_9p_qid_t {
    uint8_t type;
    uint32_t version;
    uint64_t path;
} virtio_9p_qid_t;

enum p9_qid_t {
    P9_QTDIR = 0x80,
    P9_QTAPPEND = 0x40,
    P9_QTEXCL = 0x20,
    P9_QTMOUNT = 0x10,
    P9_QTAUTH = 0x08,
    P9_QTTMP = 0x04,
    P9_QTSYMLINK = 0x02,
    P9_QTLINK = 0x01,
    P9_QTFILE = 0x00,
};

typedef struct p9_dirent {
    virtio_9p_qid_t qid;
    uint64_t offset;
    uint8_t type;
    char *name;
} p9_dirent_t;

typedef struct _virtio_9p_msg_t {
    enum virtio_9p_msg_type_t msg_type;
    uint16_t tag;

    // the 9p message structures supporting 9P2000.L
    union {
        // Tlerror
        // Rlerror
        struct {
            uint32_t ecode;
        } rlerror;

        // Tlopen
        struct {
            uint32_t fid;
            uint32_t flags;
        } tlopen;
        // Rlopen
        struct {
            virtio_9p_qid_t qid;
            uint32_t iounit;
        } rlopen;

        // Tlcreate
        struct {
            uint32_t fid;
            const char *name;
            uint32_t flags;
            uint32_t mode;
            uint32_t gid;
        } tlcreate;
        // Rlcreate
        struct {
            virtio_9p_qid_t qid;
            uint32_t iounit;
        } rlcreate;

        // Tgetattr
        struct {
            uint32_t fid;
            uint64_t request_mask;
        } tgetattr;
        // Rgetattr
        struct {
            uint64_t valid;
            virtio_9p_qid_t qid;
            uint32_t mode;
            uint32_t uid;
            uint32_t gid;
            uint64_t nlink;
            uint64_t rdev;
            uint64_t size;
            uint64_t blksize;
            uint64_t blocks;
            uint64_t atime_sec;
            uint64_t atime_nsec;
            uint64_t mtime_sec;
            uint64_t mtime_nsec;
            uint64_t ctime_sec;
            uint64_t ctime_nsec;
            uint64_t btime_sec;
            uint64_t btime_nsec;
            uint64_t gen;
            uint64_t data_version;
        } rgetattr;

        // Tversion
        struct {
            uint32_t msize;
            const char *version;
        } tversion;
        // Rversion
        struct {
            uint32_t msize;
            char *version;
        } rversion;

        // Tauth
        // Rauth

        // Rerror

        // Tflush
        // Rflush

        // Tattach
        struct {
            uint32_t fid;
            uint32_t afid;
            const char *uname;
            const char *aname;
            uint32_t n_uname;
        } tattach;
        // Rattach
        struct {
            virtio_9p_qid_t qid;
        } rattach;

        // Twalk
        struct {
            uint32_t fid;
            uint32_t newfid;
            uint16_t nwname;
            const char *wname[P9_MAXWELEM];
        } twalk;
        // Rwalk
        struct {
            uint16_t nwqid;
            virtio_9p_qid_t qid[P9_MAXWELEM];
        } rwalk;

        // Topen
        struct {
            uint32_t fid;
            uint8_t mode;
        } topen;
        // Ropen
        struct {
            virtio_9p_qid_t qid;
            uint32_t iounit;
        } ropen;

        // Tcreate
        // Rcreate

        // Tread
        struct {
            uint32_t fid;
            uint64_t offset;
            uint32_t count;
        } tread;
        // Rread
        struct {
            uint32_t count;
            uint8_t *data;
        } rread;

        // Twrite
        struct {
            uint32_t fid;
            uint64_t offset;
            uint32_t count;
            const uint8_t *data;
        } twrite;
        // Rwrite
        struct {
            uint32_t count;
        } rwrite;

        // Tclunk
        struct {
            uint32_t fid;
        } tclunk;
        // Rclunk
        struct {
        } rclunk;

        // Tremove
        struct {
            uint32_t fid;
        } tremove;
        // Rremove
        struct {
        } rremove;

        // Tstat
        // Rstat

        // Twstat
        // Rwstat

        // Treaddir
        struct {
            uint32_t fid;
            uint64_t offset;
            uint32_t count;
        } treaddir;
        // Rreaddir
        struct {
            uint32_t count;
            uint8_t *data;
        } rreaddir;

        // Tmkdir
        struct {
            uint32_t dfid;
            const char *name;
            uint32_t mode;
            uint32_t gid;
        } tmkdir;
        // Rmkdir
        struct {
            virtio_9p_qid_t qid;
        } rmkdir;
    } msg;

} virtio_9p_msg_t;

__BEGIN_CDECLS

struct virtio_device;

status_t virtio_9p_init(struct virtio_device *dev, uint32_t host_features) __NONNULL();
status_t virtio_9p_start(struct virtio_device *dev) __NONNULL();
struct virtio_device *virtio_9p_bdev_to_virtio_device(bdev_t *bdev);
struct virtio_device *virtio_get_9p_device(uint index);

status_t virtio_9p_rpc(struct virtio_device *dev, const virtio_9p_msg_t *tmsg,
                       virtio_9p_msg_t *rmsg);
void virtio_9p_msg_destroy(virtio_9p_msg_t *msg);
ssize_t p9_dirent_read(uint8_t *data, uint32_t size, p9_dirent_t *ent);
void p9_dirent_destroy(p9_dirent_t *ent);

__END_CDECLS