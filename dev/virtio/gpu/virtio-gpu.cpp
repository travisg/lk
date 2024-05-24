/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio/gpu.h>

#include <stdlib.h>
#include <lk/debug.h>
#include <assert.h>
#include <lk/trace.h>
#include <lk/compiler.h>
#include <lk/list.h>
#include <lk/err.h>
#include <string.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <dev/display.h>

#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#include "virtio-gpu.h"

#define LOCAL_TRACE 0

static enum handler_return virtio_gpu_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e);
static enum handler_return virtio_gpu_config_change_callback(struct virtio_device *dev);
static int virtio_gpu_flush_thread(void *arg);

struct virtio_gpu_dev {
    struct virtio_device *dev;

    mutex_t lock;
    event_t io_event;

    void *gpu_request;
    paddr_t gpu_request_phys;

    /* a saved copy of the display */
    struct virtio_gpu_display_one pmode;
    int pmode_id;

    /* resource id that is set as scanout */
    uint32_t display_resource_id;

    /* next resource id */
    uint32_t next_resource_id;

    event_t flush_event;

    /* framebuffer */
    void *fb;
};

static struct virtio_gpu_dev *the_gdev;

static status_t send_command_response(struct virtio_gpu_dev *gdev, const void *cmd, size_t cmd_len, void **_res, size_t res_len) {
    DEBUG_ASSERT(gdev);
    DEBUG_ASSERT(cmd);
    DEBUG_ASSERT(_res);
    DEBUG_ASSERT(cmd_len + res_len < PAGE_SIZE);

    LTRACEF("gdev %p, cmd %p, cmd_len %zu, res %p, res_len %zu\n", gdev, cmd, cmd_len, _res, res_len);

    uint16_t i;
    struct vring_desc *desc = virtio_alloc_desc_chain(gdev->dev, 0, 2, &i);
    DEBUG_ASSERT(desc);

    memcpy(gdev->gpu_request, cmd, cmd_len);

    desc->addr = gdev->gpu_request_phys;
    desc->len = cmd_len;
    desc->flags |= VRING_DESC_F_NEXT;

    /* set the second descriptor to the response with the write bit set */
    desc = virtio_desc_index_to_desc(gdev->dev, 0, desc->next);
    DEBUG_ASSERT(desc);

    void *res = (void *)((uint8_t *)gdev->gpu_request + cmd_len);
    *_res = res;
    paddr_t res_phys = gdev->gpu_request_phys + cmd_len;
    memset(res, 0, res_len);

    desc->addr = res_phys;
    desc->len = res_len;
    desc->flags = VRING_DESC_F_WRITE;

    /* submit the transfer */
    virtio_submit_chain(gdev->dev, 0, i);

    /* kick it off */
    virtio_kick(gdev->dev, 0);

    /* wait for result */
    event_wait(&gdev->io_event);

    return NO_ERROR;
}

static status_t get_display_info(struct virtio_gpu_dev *gdev) {
    status_t err;

    LTRACEF("gdev %p\n", gdev);

    DEBUG_ASSERT(gdev);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the get display info message */
    struct virtio_gpu_ctrl_hdr req;
    memset(&req, 0, sizeof(req));
    req.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

    /* send the message and get a response */
    struct virtio_gpu_resp_display_info *info;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&info, sizeof(*info));
    DEBUG_ASSERT(err == NO_ERROR);
    if (err < NO_ERROR) {
        mutex_release(&gdev->lock);
        return ERR_NOT_FOUND;
    }

    /* we got response */
    if (info->hdr.type != VIRTIO_GPU_RESP_OK_DISPLAY_INFO) {
        mutex_release(&gdev->lock);
        return ERR_NOT_FOUND;
    }

    LTRACEF("response:\n");
    for (uint i = 0; i < VIRTIO_GPU_MAX_SCANOUTS; i++) {
        if (info->pmodes[i].enabled) {
            LTRACEF("%u: x %u y %u w %u h %u flags 0x%x\n", i,
                    info->pmodes[i].r.x, info->pmodes[i].r.y, info->pmodes[i].r.width, info->pmodes[i].r.height,
                    info->pmodes[i].flags);
            if (gdev->pmode_id < 0) {
                /* save the first valid pmode we see */
                memcpy(&gdev->pmode, &info->pmodes[i], sizeof(gdev->pmode));
                gdev->pmode_id = i;
            }
        }
    }

    /* release the lock */
    mutex_release(&gdev->lock);

    return NO_ERROR;
}

static status_t allocate_2d_resource(struct virtio_gpu_dev *gdev, uint32_t *resource_id, uint32_t width, uint32_t height) {
    status_t err;

    LTRACEF("gdev %p\n", gdev);

    DEBUG_ASSERT(gdev);
    DEBUG_ASSERT(resource_id);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the request */
    struct virtio_gpu_resource_create_2d req;
    memset(&req, 0, sizeof(req));

    req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    req.resource_id = gdev->next_resource_id++;
    *resource_id = req.resource_id;
    req.format = VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM;
    req.width = width;
    req.height = height;

    /* send the command and get a response */
    struct virtio_gpu_ctrl_hdr *res;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&res, sizeof(*res));
    DEBUG_ASSERT(err == NO_ERROR);

    /* see if we got a valid response */
    LTRACEF("response type 0x%x\n", res->type);
    err = (res->type == VIRTIO_GPU_RESP_OK_NODATA) ? NO_ERROR : ERR_NO_MEMORY;

    /* release the lock */
    mutex_release(&gdev->lock);

    return err;
}

static status_t attach_backing(struct virtio_gpu_dev *gdev, uint32_t resource_id, void *ptr, size_t buf_len) {
    status_t err;

    LTRACEF("gdev %p, resource_id %u, ptr %p, buf_len %zu\n", gdev, resource_id, ptr, buf_len);

    DEBUG_ASSERT(gdev);
    DEBUG_ASSERT(ptr);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the request */
    struct {
        struct virtio_gpu_resource_attach_backing req;
        struct virtio_gpu_mem_entry mem;
    } req;
    memset(&req, 0, sizeof(req));

    req.req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
    req.req.resource_id = resource_id;
    req.req.nr_entries = 1;

    paddr_t pa;
#if WITH_KERNEL_VM
    pa = vaddr_to_paddr(ptr);
#else
    pa = (paddr_t)ptr;
#endif
    req.mem.addr = pa;
    req.mem.length = buf_len;

    /* send the command and get a response */
    struct virtio_gpu_ctrl_hdr *res;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&res, sizeof(*res));
    DEBUG_ASSERT(err == NO_ERROR);

    /* see if we got a valid response */
    LTRACEF("response type 0x%x\n", res->type);
    err = (res->type == VIRTIO_GPU_RESP_OK_NODATA) ? NO_ERROR : ERR_NO_MEMORY;

    /* release the lock */
    mutex_release(&gdev->lock);

    return err;
}

static status_t set_scanout(struct virtio_gpu_dev *gdev, uint32_t scanout_id, uint32_t resource_id, uint32_t width, uint32_t height) {
    status_t err;

    LTRACEF("gdev %p, scanout_id %u, resource_id %u, width %u, height %u\n", gdev, scanout_id, resource_id, width, height);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the request */
    struct virtio_gpu_set_scanout req;
    memset(&req, 0, sizeof(req));

    req.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    req.r.x = req.r.y = 0;
    req.r.width = width;
    req.r.height = height;
    req.scanout_id = scanout_id;
    req.resource_id = resource_id;

    /* send the command and get a response */
    struct virtio_gpu_ctrl_hdr *res;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&res, sizeof(*res));
    DEBUG_ASSERT(err == NO_ERROR);

    /* see if we got a valid response */
    LTRACEF("response type 0x%x\n", res->type);
    err = (res->type == VIRTIO_GPU_RESP_OK_NODATA) ? NO_ERROR : ERR_NO_MEMORY;

    /* release the lock */
    mutex_release(&gdev->lock);

    return err;
}

static status_t flush_resource(struct virtio_gpu_dev *gdev, uint32_t resource_id, uint32_t width, uint32_t height) {
    status_t err;

    LTRACEF("gdev %p, resource_id %u, width %u, height %u\n", gdev, resource_id, width, height);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the request */
    struct virtio_gpu_resource_flush req;
    memset(&req, 0, sizeof(req));

    req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
    req.r.x = req.r.y = 0;
    req.r.width = width;
    req.r.height = height;
    req.resource_id = resource_id;

    /* send the command and get a response */
    struct virtio_gpu_ctrl_hdr *res;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&res, sizeof(*res));
    DEBUG_ASSERT(err == NO_ERROR);

    /* see if we got a valid response */
    LTRACEF("response type 0x%x\n", res->type);
    err = (res->type == VIRTIO_GPU_RESP_OK_NODATA) ? NO_ERROR : ERR_NO_MEMORY;

    /* release the lock */
    mutex_release(&gdev->lock);

    return err;
}

static status_t transfer_to_host_2d(struct virtio_gpu_dev *gdev, uint32_t resource_id, uint32_t width, uint32_t height) {
    status_t err;

    LTRACEF("gdev %p, resource_id %u, width %u, height %u\n", gdev, resource_id, width, height);

    /* grab a lock to keep this single message at a time */
    mutex_acquire(&gdev->lock);

    /* construct the request */
    struct virtio_gpu_transfer_to_host_2d req;
    memset(&req, 0, sizeof(req));

    req.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
    req.r.x = req.r.y = 0;
    req.r.width = width;
    req.r.height = height;
    req.offset = 0;
    req.resource_id = resource_id;

    /* send the command and get a response */
    struct virtio_gpu_ctrl_hdr *res;
    err = send_command_response(gdev, &req, sizeof(req), (void **)&res, sizeof(*res));
    DEBUG_ASSERT(err == NO_ERROR);

    /* see if we got a valid response */
    LTRACEF("response type 0x%x\n", res->type);
    err = (res->type == VIRTIO_GPU_RESP_OK_NODATA) ? NO_ERROR : ERR_NO_MEMORY;

    /* release the lock */
    mutex_release(&gdev->lock);

    return err;
}

status_t virtio_gpu_start(struct virtio_device *dev) {
    status_t err;

    LTRACEF("dev %p\n", dev);

    struct virtio_gpu_dev *gdev = (struct virtio_gpu_dev *)dev->priv;

    /* get the display info and see if we find a valid pmode */
    err = get_display_info(gdev);
    if (err < 0) {
        LTRACEF("failed to get display info\n");
        return err;
    }

    if (gdev->pmode_id < 0) {
        LTRACEF("we failed to find a pmode, exiting\n");
        return ERR_NOT_FOUND;
    }

    /* allocate a resource */
    err = allocate_2d_resource(gdev, &gdev->display_resource_id, gdev->pmode.r.width, gdev->pmode.r.height);
    if (err < 0) {
        LTRACEF("failed to allocate 2d resource\n");
        return err;
    }

    /* attach a backing store to the resource */
    size_t len = gdev->pmode.r.width * gdev->pmode.r.height * 4;
#if WITH_KERNEL_VM
    gdev->fb = pmm_alloc_kpages(ROUNDUP(len, PAGE_SIZE) / PAGE_SIZE, NULL);
#else
    gdev->fb = memalign(PAGE_SIZE, ROUNDUP(len, PAGE_SIZE));
#endif
    if (!gdev->fb) {
        TRACEF("failed to allocate framebuffer, wanted 0x%zx bytes\n", len);
        return ERR_NO_MEMORY;
    }

    printf("virtio-gpu: framebuffer at %p, 0x%zx bytes\n", gdev->fb, len);

    err = attach_backing(gdev, gdev->display_resource_id, gdev->fb, len);
    if (err < 0) {
        LTRACEF("failed to attach backing store\n");
        return err;
    }

    /* attach this resource as a scanout */
    err = set_scanout(gdev, gdev->pmode_id, gdev->display_resource_id, gdev->pmode.r.width, gdev->pmode.r.height);
    if (err < 0) {
        LTRACEF("failed to set scanout\n");
        return err;
    }

    /* create the flush thread */
    thread_t *t;
    t = thread_create("virtio gpu flusher", &virtio_gpu_flush_thread, (void *)gdev, HIGH_PRIORITY, DEFAULT_STACK_SIZE);
    thread_detach_and_resume(t);

    /* kick it once */
    event_signal(&gdev->flush_event, true);

    LTRACE_EXIT;

    return NO_ERROR;
}


static void dump_gpu_config(const volatile struct virtio_gpu_config *config) {
    LTRACEF("events_read 0x%x\n", config->events_read);
    LTRACEF("events_clear 0x%x\n", config->events_clear);
    LTRACEF("num_scanouts 0x%x\n", config->num_scanouts);
    LTRACEF("reserved 0x%x\n", config->reserved);
}

status_t virtio_gpu_init(struct virtio_device *dev, uint32_t host_features) {
    LTRACEF("dev %p, host_features 0x%x\n", dev, host_features);

    /* allocate a new gpu device */
    struct virtio_gpu_dev *gdev = (virtio_gpu_dev *)malloc(sizeof(struct virtio_gpu_dev));
    if (!gdev)
        return ERR_NO_MEMORY;

    mutex_init(&gdev->lock);
    event_init(&gdev->io_event, false, EVENT_FLAG_AUTOUNSIGNAL);
    event_init(&gdev->flush_event, false, EVENT_FLAG_AUTOUNSIGNAL);

    gdev->dev = dev;
    dev->priv = gdev;

    gdev->pmode_id = -1;
    gdev->next_resource_id = 1;

    /* allocate memory for a gpu request */
#if WITH_KERNEL_VM
    gdev->gpu_request = pmm_alloc_kpage();
    gdev->gpu_request_phys = vaddr_to_paddr(gdev->gpu_request);
#else
    gdev->gpu_request = malloc(sizeof(struct virtio_gpu_resp_display_info)); // XXX get size better
    gdev->gpu_request_phys = (paddr_t)gdev->gpu_request;
#endif

    /* make sure the device is reset */
    virtio_reset_device(dev);

    volatile struct virtio_gpu_config *config = (struct virtio_gpu_config *)dev->config_ptr;
    dump_gpu_config(config);

    /* ack and set the driver status bit */
    virtio_status_acknowledge_driver(dev);

    // XXX check features bits and ack/nak them

    /* allocate a virtio ring */
    virtio_alloc_ring(dev, 0, 16);

    /* set our irq handler */
    dev->irq_driver_callback = &virtio_gpu_irq_driver_callback;
    dev->config_change_callback = &virtio_gpu_config_change_callback;

    /* set DRIVER_OK */
    virtio_status_driver_ok(dev);

    /* save the main device we've found */
    the_gdev = gdev;

    printf("found virtio gpu device\n");

    return NO_ERROR;
}

static enum handler_return virtio_gpu_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e) {
    struct virtio_gpu_dev *gdev = (struct virtio_gpu_dev *)dev->priv;

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
    event_signal(&gdev->io_event, false);

    return INT_RESCHEDULE;
}

static enum handler_return virtio_gpu_config_change_callback(struct virtio_device *dev) {
    struct virtio_gpu_dev *gdev = (struct virtio_gpu_dev *)dev->priv;

    LTRACEF("gdev %p\n", gdev);

    volatile struct virtio_gpu_config *config = (struct virtio_gpu_config *)dev->config_ptr;
    dump_gpu_config(config);

    return INT_RESCHEDULE;
}

static int virtio_gpu_flush_thread(void *arg) {
    struct virtio_gpu_dev *gdev = (struct virtio_gpu_dev *)arg;
    status_t err;

    for (;;) {
        event_wait(&gdev->flush_event);

        /* transfer to host 2d */
        err = transfer_to_host_2d(gdev, gdev->display_resource_id, gdev->pmode.r.width, gdev->pmode.r.height);
        if (err < 0) {
            LTRACEF("failed to flush resource\n");
            continue;
        }

        /* resource flush */
        err = flush_resource(gdev, gdev->display_resource_id, gdev->pmode.r.width, gdev->pmode.r.height);
        if (err < 0) {
            LTRACEF("failed to flush resource\n");
            continue;
        }
    }

    return 0;
}

static void virtio_gpu_gfx_flush(uint starty, uint endy) {
    event_signal(&the_gdev->flush_event, !arch_ints_disabled());
}

status_t display_get_framebuffer(struct display_framebuffer *fb) {
    DEBUG_ASSERT(fb);
    memset(fb, 0, sizeof(*fb));

    if (!the_gdev)
        return ERR_NOT_FOUND;

    fb->image.pixels = the_gdev->fb;
    fb->image.format = IMAGE_FORMAT_RGB_x888;
    fb->image.width = the_gdev->pmode.r.width;
    fb->image.height = the_gdev->pmode.r.height;
    fb->image.stride = fb->image.width;
    fb->image.rowbytes = fb->image.width * 4;
    fb->flush = virtio_gpu_gfx_flush;
    fb->format = DISPLAY_FORMAT_RGB_x888;

    return NO_ERROR;
}

status_t display_get_info(struct display_info *info) {
    DEBUG_ASSERT(info);
    memset(info, 0, sizeof(*info));

    if (!the_gdev)
        return ERR_NOT_FOUND;

    info->format = DISPLAY_FORMAT_RGB_x888;
    info->width = the_gdev->pmode.r.width;
    info->height = the_gdev->pmode.r.height;

    return NO_ERROR;
}

status_t display_present(struct display_image *image, uint starty, uint endy) {
    TRACEF("display_present - not implemented");
    DEBUG_ASSERT(false);
    return NO_ERROR;
}
