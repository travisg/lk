/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio/block.h>

#include <stdlib.h>
#include <lk/debug.h>
#include <assert.h>
#include <lk/trace.h>
#include <lk/compiler.h>
#include <lk/list.h>
#include <lk/err.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <lib/bio.h>
#include <inttypes.h>

#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#define LOCAL_TRACE 0

struct virtio_blk_config {
    uint64_t capacity;
    uint32_t size_max;
    uint32_t seg_max;
    struct virtio_blk_geometry {
        uint16_t cylinders;
        uint8_t heads;
        uint8_t sectors;
    } geometry;
    uint32_t blk_size;
    struct virtio_blk_topology {
        uint8_t physical_block_exp;
        uint8_t alignment_offset;
        uint16_t min_io_size;
        uint32_t opt_io_size;
    } topology;
    uint8_t writeback;
    uint8_t unused[3];
    uint32_t max_discard_sectors;
    uint32_t max_discard_seq;
    uint32_t discard_sector_alignment;
    uint32_t max_write_zeroes_sectors;
    uint32_t max_write_zeroes_seq;
    uint8_t write_zeros_may_unmap;
    uint8_t unused1[3];
    uint32_t max_secure_erase_sectors;
    uint32_t max_secure_erase_seg;
    uint32_t secure_erase_sector_alignment;
    struct virtio_blk_zoned_characteristics {
        uint32_t zone_sectors;
        uint32_t max_open_zones;
        uint32_t max_active_zones;
        uint32_t max_append_sectors;
        uint32_t write_granularity;
        uint8_t model;
        uint8_t unused2[3];
    } zoned;
};
STATIC_ASSERT(sizeof(struct virtio_blk_config) == 96);

struct virtio_blk_req {
    uint32_t type;
    uint32_t ioprio; // v1.3 says this is 'reserved'
    uint64_t sector;
};
STATIC_ASSERT(sizeof(struct virtio_blk_req) == 16);

struct virtio_blk_discard_write_zeroes {
    uint64_t sector;
    uint32_t num_sectors;
    struct {
        uint32_t unmap:1;
        uint32_t reserved:31;
    } flags;
};
STATIC_ASSERT(sizeof(struct virtio_blk_req) == 16);

#define VIRTIO_BLK_F_BARRIER  (1<<0) // legacy
#define VIRTIO_BLK_F_SIZE_MAX (1<<1)
#define VIRTIO_BLK_F_SEG_MAX  (1<<2)
#define VIRTIO_BLK_F_GEOMETRY (1<<4)
#define VIRTIO_BLK_F_RO       (1<<5)
#define VIRTIO_BLK_F_BLK_SIZE (1<<6)
#define VIRTIO_BLK_F_SCSI     (1<<7) // legacy
#define VIRTIO_BLK_F_FLUSH    (1<<9)
#define VIRTIO_BLK_F_TOPOLOGY (1<<10)
#define VIRTIO_BLK_F_CONFIG_WCE (1<<11)
#define VIRTIO_BLK_F_DISCARD  (1<<13)
#define VIRTIO_BLK_F_WRITE_ZEROES (1<<14)
#define VIRTIO_BLK_F_LIFETIME (1<<15)
#define VIRTIO_BLK_F_SECURE_ERASE (1<<16)
#define VIRTIO_BLK_F_ZONED    (1<<17)

#define VIRTIO_BLK_T_IN           0
#define VIRTIO_BLK_T_OUT          1
#define VIRTIO_BLK_T_FLUSH        4
#define VIRTIO_BLK_T_GET_ID       8
#define VIRTIO_BLK_T_GET_LIFETIME 10
#define VIRTIO_BLK_T_DISCARD      11
#define VIRTIO_BLK_T_WRITE_ZEROES 13
#define VIRTIO_BLK_T_SECURE_ERASE 13

#define VIRTIO_BLK_S_OK         0
#define VIRTIO_BLK_S_IOERR      1
#define VIRTIO_BLK_S_UNSUPP     2

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e);
static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count);
static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block, uint count);

struct virtio_block_dev {
    struct virtio_device *dev;

    mutex_t lock;
    event_t io_event;

    /* bio block device */
    bdev_t bdev;

    /* our negotiated guest features */
    uint32_t guest_features;

    /* one blk_req structure for io, not crossing a page boundary */
    struct virtio_blk_req *blk_req;
    paddr_t blk_req_phys;

    /* one uint8_t response word */
    uint8_t blk_response;
    paddr_t blk_response_phys;
};

static void dump_feature_bits(const char *name, uint32_t feature) {
    printf("virtio-block %s features (%#x):", name, feature);
    if (feature & VIRTIO_BLK_F_BARRIER) printf(" BARRIER");
    if (feature & VIRTIO_BLK_F_SIZE_MAX) printf(" SIZE_MAX");
    if (feature & VIRTIO_BLK_F_SEG_MAX) printf(" SEG_MAX");
    if (feature & VIRTIO_BLK_F_GEOMETRY) printf(" GEOMETRY");
    if (feature & VIRTIO_BLK_F_RO) printf(" RO");
    if (feature & VIRTIO_BLK_F_BLK_SIZE) printf(" BLK_SIZE");
    if (feature & VIRTIO_BLK_F_SCSI) printf(" SCSI");
    if (feature & VIRTIO_BLK_F_FLUSH) printf(" FLUSH");
    if (feature & VIRTIO_BLK_F_TOPOLOGY) printf(" TOPOLOGY");
    if (feature & VIRTIO_BLK_F_CONFIG_WCE) printf(" CONFIG_WCE");
    if (feature & VIRTIO_BLK_F_DISCARD) printf(" DISCARD");
    if (feature & VIRTIO_BLK_F_WRITE_ZEROES) printf(" WRITE_ZEROES");
    if (feature & VIRTIO_BLK_F_LIFETIME) printf(" LIFETIME");
    if (feature & VIRTIO_BLK_F_SECURE_ERASE) printf(" SECURE_ERASE");
    if (feature & VIRTIO_BLK_F_ZONED) printf(" ZONED");
    printf("\n");
}

status_t virtio_block_init(struct virtio_device *dev, uint32_t host_features) {
    LTRACEF("dev %p, host_features %#x\n", dev, host_features);

    /* allocate a new block device */
    struct virtio_block_dev *bdev = (virtio_block_dev *)malloc(sizeof(struct virtio_block_dev));
    if (!bdev)
        return ERR_NO_MEMORY;

    mutex_init(&bdev->lock);
    event_init(&bdev->io_event, false, EVENT_FLAG_AUTOUNSIGNAL);

    bdev->dev = dev;
    dev->priv = bdev;

    bdev->blk_req = (virtio_blk_req *)memalign(sizeof(struct virtio_blk_req), sizeof(struct virtio_blk_req));
#if WITH_KERNEL_VM
    bdev->blk_req_phys = vaddr_to_paddr(bdev->blk_req);
#else
    bdev->blk_req_phys = (uint64_t)(uintptr_t)bdev->blk_req;
#endif
    LTRACEF("blk_req structure at %p (%#lx phys)\n", bdev->blk_req, bdev->blk_req_phys);

#if WITH_KERNEL_VM
    bdev->blk_response_phys = vaddr_to_paddr(&bdev->blk_response);
#else
    bdev->blk_response_phys = (uint64_t)(uintptr_t)&bdev->blk_response;
#endif

    /* make sure the device is reset */
    virtio_reset_device(dev);

    volatile struct virtio_blk_config *config = (struct virtio_blk_config *)dev->config_ptr;

    LTRACEF("capacity %" PRIx64 "\n", config->capacity);
    LTRACEF("size_max %#x\n", config->size_max);
    LTRACEF("seg_max  %#x\n", config->seg_max);
    LTRACEF("blk_size %#x\n", config->blk_size);

    /* ack and set the driver status bit */
    virtio_status_acknowledge_driver(dev);

    /* check features bits and ack/nak them */
    bdev->guest_features = host_features;

    /* keep the features we understand or can tolerate */
    bdev->guest_features &= (VIRTIO_BLK_F_SIZE_MAX |
                             VIRTIO_BLK_F_BLK_SIZE |
                             VIRTIO_BLK_F_GEOMETRY |
                             VIRTIO_BLK_F_BLK_SIZE |
                             VIRTIO_BLK_F_TOPOLOGY |
                             VIRTIO_BLK_F_DISCARD |
                             VIRTIO_BLK_F_WRITE_ZEROES);
    virtio_set_guest_features(dev, 0, bdev->guest_features);

    /* TODO: handle a RO feature */

    /* allocate a virtio ring */
    virtio_alloc_ring(dev, 0, 256);

    /* set our irq handler */
    dev->irq_driver_callback = &virtio_block_irq_driver_callback;

    /* set DRIVER_OK */
    virtio_status_driver_ok(dev);

    /* construct the block device */
    static uint8_t found_index = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "virtio%u", found_index++);
    bio_initialize_bdev(&bdev->bdev, buf,
                        config->blk_size, config->capacity,
                        0, NULL, BIO_FLAGS_NONE);

    /* override our block device hooks */
    bdev->bdev.read_block = &virtio_bdev_read_block;
    bdev->bdev.write_block = &virtio_bdev_write_block;

    bio_register_device(&bdev->bdev);

    printf("virtio-block found device of size %" PRIu64 "\n", config->capacity * config->blk_size);

    /* dump feature bits */
    dump_feature_bits("host", host_features);
    dump_feature_bits("guest", bdev->guest_features);
    printf("\tsize_max %u seg_max %u\n", config->size_max, config->seg_max);
    if (host_features & VIRTIO_BLK_F_GEOMETRY) {
        printf("\tgeometry: cyl %u head %u sector %u\n", config->geometry.cylinders, config->geometry.heads, config->geometry.sectors);
    }
    if (host_features & VIRTIO_BLK_F_BLK_SIZE) {
        printf("\tblock_size: %u\n", config->blk_size);
    }
    if (host_features & VIRTIO_BLK_F_TOPOLOGY) {
        printf("\ttopology: block exp %u alignment_offset %u min_io_size %u opt_io_size %u\n",
               config->topology.physical_block_exp, config->topology.alignment_offset,
               config->topology.min_io_size, config->topology.opt_io_size);
    }
    if (host_features & VIRTIO_BLK_F_DISCARD) {
        printf("\tdiscard: max sectors %u max sequence %u alignment %u\n",
               config->max_discard_sectors, config->max_discard_sectors, config->discard_sector_alignment);
    }
    if (host_features & VIRTIO_BLK_F_WRITE_ZEROES) {
        printf("\twrite zeroes: max sectors %u max sequence %u may unmap %u\n",
               config->max_write_zeroes_sectors, config->max_write_zeroes_seq, config->write_zeros_may_unmap);
    }

    return NO_ERROR;
}

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e) {
    struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

    LTRACEF("dev %p, ring %u, e %p, id %u, len %u\n", dev, ring, e, e->id, e->len);

    /* parse our descriptor chain, add back to the free queue */
    uint16_t i = e->id;
    for (;;) {
        int next;
        struct vring_desc *desc = virtio_desc_index_to_desc(dev, ring, i);

        //virtio_dump_desc(desc);

        if (desc->flags & VRING_DESC_F_NEXT) {
            next = desc->next;
        } else {
            /* end of chain */
            next = -1;
        }

        virtio_free_desc(dev, ring, i);

        if (next < 0)
            break;
        i = next;
    }

    /* signal our event */
    event_signal(&bdev->io_event, false);

    return INT_RESCHEDULE;
}

ssize_t virtio_block_read_write(struct virtio_device *dev, void *buf, const off_t offset, const size_t len, const bool write) {
    struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

    uint16_t i;
    struct vring_desc *desc;

    LTRACEF("dev %p, buf %p, offset 0x%llx, len %zu\n", dev, buf, offset, len);

    mutex_acquire(&bdev->lock);

    /* set up the request */
    bdev->blk_req->type = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
    bdev->blk_req->ioprio = 0;
    bdev->blk_req->sector = offset / 512;
    LTRACEF("blk_req type %u ioprio %u sector %llu\n",
            bdev->blk_req->type, bdev->blk_req->ioprio, bdev->blk_req->sector);

    /* put together a transfer */
    desc = virtio_alloc_desc_chain(dev, 0, 3, &i);
    LTRACEF("after alloc chain desc %p, i %u\n", desc, i);

    // XXX not cache safe.
    // At the moment only tested on arm qemu, which doesn't emulate cache.

    /* set up the descriptor pointing to the head */
    desc->addr = bdev->blk_req_phys;
    desc->len = sizeof(struct virtio_blk_req);
    desc->flags |= VRING_DESC_F_NEXT;

    /* set up the descriptor pointing to the buffer */
    desc = virtio_desc_index_to_desc(dev, 0, desc->next);
#if WITH_KERNEL_VM
    /* translate the first buffer */
    vaddr_t va = (vaddr_t)buf;
    paddr_t pa = vaddr_to_paddr((void *)va);
    desc->addr = (uint64_t)pa;
    /* desc->len is filled in below */
#else
    /* non VM world simply queues a single buffer that transfers the whole thing */
    desc->addr = (uint64_t)(uintptr_t)buf;
    desc->len = len;
#endif
    desc->flags |= write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
    desc->flags |= VRING_DESC_F_NEXT;

#if WITH_KERNEL_VM
    /* see if we need to add more descriptors due to scatter gather */
    paddr_t next_pa = PAGE_ALIGN(pa + 1);
    desc->len = MIN(next_pa - pa, len);
    LTRACEF("first descriptor va 0x%lx desc->addr 0x%llx desc->len %u\n", va, desc->addr, desc->len);

    size_t remaining_len = len;
    remaining_len -= desc->len;
    while (remaining_len > 0) {
        /* amount of source buffer handled by this iteration of the loop */
        size_t len_tohandle = MIN(remaining_len, PAGE_SIZE);

        /* translate the next page in the buffer */
        va = PAGE_ALIGN(va + 1);
        pa = vaddr_to_paddr((void *)va);
        LTRACEF("va now 0x%lx, pa 0x%lx, next_pa 0x%lx, remaining len %zu\n", va, pa, next_pa, remaining_len);

        /* is the new translated physical address contiguous to the last one? */
        if (next_pa == pa) {
            /* we can simply extend the previous descriptor by another page */
            LTRACEF("extending last one by %zu bytes\n", len_tohandle);
            desc->len += len_tohandle;
        } else {
            /* new physical page needed, allocate a new descriptor and start again */
            uint16_t next_i = virtio_alloc_desc(dev, 0);
            struct vring_desc *next_desc = virtio_desc_index_to_desc(dev, 0, next_i);
            DEBUG_ASSERT(next_desc);

            LTRACEF("doesn't extend, need new desc, allocated desc %i (%p)\n", next_i, next_desc);

            /* fill this descriptor in and put it after the last one but before the response descriptor */
            next_desc->addr = (uint64_t)pa;
            next_desc->len = len_tohandle;
            next_desc->flags = write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
            next_desc->flags |= VRING_DESC_F_NEXT;
            next_desc->next = desc->next;
            desc->next = next_i;

            desc = next_desc;
        }
        remaining_len -= len_tohandle;
        next_pa += PAGE_SIZE;
    }
#endif

    /* set up the descriptor pointing to the response */
    desc = virtio_desc_index_to_desc(dev, 0, desc->next);
    desc->addr = bdev->blk_response_phys;
    desc->len = 1;
    desc->flags = VRING_DESC_F_WRITE;

    /* submit the transfer */
    virtio_submit_chain(dev, 0, i);

    /* kick it off */
    virtio_kick(dev, 0);

    /* wait for the transfer to complete */
    event_wait(&bdev->io_event);

    LTRACEF("status 0x%hhx\n", bdev->blk_response);

    /* TODO: handle transfer errors and return error */

    mutex_release(&bdev->lock);

    return len;
}

static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count) {
    struct virtio_block_dev *dev = containerof(bdev, struct virtio_block_dev, bdev);

    LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

    ssize_t result = virtio_block_read_write(dev->dev, buf, (off_t)block * dev->bdev.block_size,
                     count * dev->bdev.block_size, false);
    return result;
}

static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block, uint count) {
    struct virtio_block_dev *dev = containerof(bdev, struct virtio_block_dev, bdev);

    LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

    ssize_t result = virtio_block_read_write(dev->dev, (void *)buf, (off_t)block * dev->bdev.block_size,
                     count * dev->bdev.block_size, true);
    return result;
}

