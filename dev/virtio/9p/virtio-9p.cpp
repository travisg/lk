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
#include <assert.h>
#include <dev/virtio/9p.h>
#include <dev/virtio/virtio-device.h>
#include <kernel/event.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/list.h>
#include <lk/trace.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"

#define LOCAL_TRACE 0

struct virtio_9p_config {
    uint16_t tag_len;
    uint8_t tag[];
};

#define VIRTIO_9P_MOUNT_TAG                   (1<<0)

static enum handler_return virtio_9p_irq_driver_callback(
    virtio_device *dev, uint ring, const struct vring_used_elem *e);

static struct list_node p9_devices = LIST_INITIAL_VALUE(p9_devices);

static void dump_feature_bits(uint32_t feature)
{
    LTRACEF("virtio-9p host features (0x%x):", feature);
    if (feature & VIRTIO_9P_MOUNT_TAG) LTRACEF(" MOUNT_TAG");
    LTRACEF("\n");
}

status_t virtio_9p_init(virtio_device *dev, uint32_t host_features)
{
    dump_feature_bits(host_features);

    /* allocate a new 9p device */
    auto *p9dev = (virtio_9p_dev *)calloc(1, sizeof(struct virtio_9p_dev));
    if (!p9dev)
        return ERR_NO_MEMORY;

    p9dev->dev = dev;
    dev->priv_ = p9dev;
    p9dev->lock = SPIN_LOCK_INITIAL_VALUE;
    // Assuming there can be only one outstanding request.
    p9dev->req.status = P9_REQ_S_UNKNOWN;
    mutex_init(&p9dev->req_lock);
    p9dev->msize = VIRTIO_9P_DEFAULT_MSIZE;

    // Add the 9p device to the device list
    list_add_tail(&p9_devices, &p9dev->list);

    /* make sure the device is reset */
    dev->virtio_reset_device();

    p9dev->config = (struct virtio_9p_config *)dev->config_ptr_;
#if LOCAL_TRACE
    LTRACEF("tag_len: %u\n", p9dev->config->tag_len);
    LTRACEF("tag: ");
    for (int i = 0; i < p9dev->config->tag_len; ++i) {
        printf("%c", p9dev->config->tag[i]);
    }
    printf("\n");
#endif

    /* ack and set the driver status bit */
    dev->virtio_status_acknowledge_driver();

    dev->virtio_alloc_ring(VIRTIO_9P_RING_IDX, VIRTIO_9P_RING_SIZE);

    /* set our irq handler */
    dev->irq_driver_callback_ = &virtio_9p_irq_driver_callback;

    /* set DRIVER_OK */
    dev->virtio_status_driver_ok();

    // register a fake block device
    static uint8_t found_index = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "v9p%u", found_index++);
    bio_initialize_bdev(&p9dev->bdev, buf, 1, 0,
                        0, NULL, BIO_FLAGS_NONE);

    // override our block device hooks
    p9dev->bdev.read_block = NULL;
    p9dev->bdev.write_block = NULL;

    bio_register_device(&p9dev->bdev);

    return NO_ERROR;
}

status_t virtio_9p_start(virtio_device *dev)
{
    auto *p9dev = (virtio_9p_dev *)dev->priv_;
    status_t ret;

    // connect to the 9p server with 9P2000.L
    virtio_9p_msg_t tver = {
        .msg_type = P9_TVERSION,
        .tag = P9_TAG_NOTAG,
        .msg = {
            .tversion = { .msize = p9dev->msize, .version = "9P2000.L"}
        }
    };
    virtio_9p_msg_t rver = {};

    if ((ret = virtio_9p_rpc(dev, &tver, &rver)) != NO_ERROR)
        return ret;

    // assert the server support 9P2000.L version
    ASSERT(strcmp(rver.msg.rversion.version, "9P2000.L") == 0);
    p9dev->msize = rver.msg.rversion.msize;

    virtio_9p_msg_destroy(&rver);

    return NO_ERROR;
}

static enum handler_return virtio_9p_irq_driver_callback(
    virtio_device *dev, uint ring, const vring_used_elem *e)
{
    auto *p9dev = (virtio_9p_dev *)dev->priv_;
    uint16_t id = e->id;
    uint16_t id_next;
    vring_desc *desc = dev->virtio_desc_index_to_desc(ring, id);
    struct p9_req *req = &p9dev->req;

    LTRACEF("dev %p, ring %u, e %p, id %u, len %u\n", dev, ring, e, e->id, e->len);
#if LOCAL_TRACE
    virtio_dump_desc(desc);
#endif

    ASSERT(req->status == P9_REQ_S_SENT);
    ASSERT(desc);
    ASSERT(desc->flags & VRING_DESC_F_NEXT);

    spin_lock(&p9dev->lock);

    // drop the T-message desc
    id_next = desc->next;
    desc = dev->virtio_desc_index_to_desc(VIRTIO_9P_RING_IDX, id_next);
#if LOCAL_TRACE
    virtio_dump_desc(desc);
#endif
    req->rc.size = e->len;
    req->status = P9_REQ_S_RECEIVED;

    // free the desc
    dev->virtio_free_desc(ring, id);
    dev->virtio_free_desc(ring, id_next);

    spin_unlock(&p9dev->lock);

    /* wake up the rpc */
    event_signal(&req->io_event, false);

    return INT_RESCHEDULE;
}

static struct virtio_9p_dev *bdev_to_virtio_9p_dev(bdev_t *bdev)
{
    return containerof(bdev, struct virtio_9p_dev, bdev);
}

struct virtio_device *virtio_9p_bdev_to_virtio_device(bdev_t *bdev)
{
    return bdev_to_virtio_9p_dev(bdev)->dev;
}

struct virtio_device *virtio_get_9p_device(uint index)
{
    struct virtio_9p_dev *p9dev;
    uint count = 0;

    list_for_every_entry(&p9_devices, p9dev, struct virtio_9p_dev, list) {
        if (count == index)
            return p9dev->dev;
        count++;
    }

    return NULL;
}
