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
#include <dev/virtio.h>
#include <dev/virtio/virtio-device.h>
#include <dev/virtio/9p.h>
#include <kernel/event.h>
#include <kernel/vm.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/trace.h>

#include "protocol.h"

#define LOCAL_TRACE 0

static status_t pdu_init(struct p9_fcall *pdu, size_t size)
{
    vmm_alloc_contiguous(vmm_get_kernel_aspace(), "virtio_9p_pdu", size,
                         (void **)&pdu->sdata, 0, 0,
                         ARCH_MMU_FLAG_UNCACHED_DEVICE);
    if (!pdu->sdata)
        return ERR_NO_MEMORY;
    pdu->capacity = size;
    return NO_ERROR;
}

static void pdu_fini(struct p9_fcall *pdu)
{
    if (pdu->sdata)
        vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t)pdu->sdata);
    pdu->sdata = NULL;
    pdu->capacity = 0;
}

static void pdu_reset(struct p9_fcall *pdu)
{
    pdu->offset = 0;
    pdu->size = 0;
}

static status_t p9_req_prepare(struct p9_req *req,
                               const virtio_9p_msg_t *tmsg)
{
    struct virtio_9p_dev *p9dev = containerof(req, struct virtio_9p_dev, req);
    status_t ret = NO_ERROR;

    if ((ret = pdu_init(&req->tc, p9dev->msize)) != NO_ERROR) {
        goto err;
    }

    if ((ret = pdu_init(&req->rc, p9dev->msize)) != NO_ERROR) {
        goto err;
    }

    pdu_reset(&req->tc);
    pdu_reset(&req->rc);

    event_init(&req->io_event, false, EVENT_FLAG_AUTOUNSIGNAL);
    req->status = P9_REQ_S_INITIALIZED;

    // fill 9p header
    if (pdu_writed(&req->tc, 0) != NO_ERROR) {
        ret = ERR_IO;
        goto err;
    }
    if (pdu_writeb(&req->tc, tmsg->msg_type) != NO_ERROR) {
        ret = ERR_IO;
        goto err;
    }
    if (pdu_writew(&req->tc, tmsg->tag) != NO_ERROR) {
        ret = ERR_IO;
        goto err;
    }

    return NO_ERROR;
err:
    pdu_fini(&req->tc);
    pdu_fini(&req->rc);
    return ret;
}

static void p9_req_release(struct p9_req *req)
{
    req->status = P9_REQ_S_UNKNOWN;
    event_destroy(&req->io_event);

    pdu_fini(&req->tc);
    pdu_fini(&req->rc);
}

static status_t p9_req_finalize(struct p9_req *req)
{
    uint32_t size = req->tc.size;
    status_t ret;

    pdu_reset(&req->tc);
    ret = pdu_writed(&req->tc, size);
    req->tc.size = size;
#if LOCAL_TRACE >= 2
    LTRACEF("req->tc.sdata (%p) size (%u)\n", req->tc.sdata, size);
    hexdump8(req->tc.sdata, size);
#endif

    return ret;
}

static void p9_req_receive(struct p9_req *req,
                           virtio_9p_msg_t *rmsg)
{
    pdu_readd(&req->rc);
    rmsg->msg_type = (virtio_9p_msg_type_t)pdu_readb(&req->rc);
    rmsg->tag = pdu_readw(&req->rc);
#if LOCAL_TRACE >= 2
    LTRACEF("req->rc.sdata (%p) req->rc.size (%u)\n", req->rc.sdata,
            req->rc.size);
    hexdump8(req->rc.sdata, req->rc.size);
#endif
}

static void virtio_9p_req_send(struct virtio_9p_dev *p9dev,
                               struct p9_req *req)
{
    struct virtio_device *dev = p9dev->dev;
    struct vring_desc *desc;
    uint16_t idx;

    spin_lock_saved_state_t state;
    spin_lock_irqsave(&p9dev->lock, state);

    desc = dev->virtio_alloc_desc_chain(VIRTIO_9P_RING_IDX, 2, &idx);

    desc->len = req->tc.size;
    desc->addr = vaddr_to_paddr(req->tc.sdata);
    desc->flags |= VRING_DESC_F_NEXT;
#if LOCAL_TRACE > 2
    LTRACEF("desc (%p)\n", desc);
    virtio_dump_desc(desc);
#endif

    desc = dev->virtio_desc_index_to_desc(VIRTIO_9P_RING_IDX, desc->next);
    desc->len = req->rc.capacity;
    desc->addr = vaddr_to_paddr(req->rc.sdata);
    desc->flags |= VRING_DESC_F_WRITE;
#if LOCAL_TRACE > 2
    LTRACEF("desc (%p)\n", desc);
    virtio_dump_desc(desc);
#endif

    req->status = P9_REQ_S_SENT;

    /* submit the transfer */
    dev->virtio_submit_chain(VIRTIO_9P_RING_IDX, idx);

    /* kick it off */
    dev->virtio_kick(VIRTIO_9P_RING_IDX);

    spin_unlock_irqrestore(&p9dev->lock, state);
}

status_t virtio_9p_rpc(struct virtio_device *dev, const virtio_9p_msg_t *tmsg,
                       virtio_9p_msg_t *rmsg)
{
    LTRACEF("dev (%p) tmsg (%p) rmsg (%p)\n", dev, tmsg, rmsg);

    auto *p9dev = (virtio_9p_dev *)dev->priv_;
    struct p9_req *req = &p9dev->req;
    status_t ret;

    if (!tmsg || !rmsg) {
        return ERR_INVALID_ARGS;
    }

    // Since we allow only one outstanding request for now, we have a 9p device
    // level lock for restricting only one rpc can be executed at a time. One
    // day if we can support multiple outstanding requests, we should move the
    // lock into the request allocation phase.
    mutex_acquire(&p9dev->req_lock);

    // prepare the message header
    ret = p9_req_prepare(req, tmsg);
    if (ret != NO_ERROR) {
        goto req_unlock;
    }

    // setup the T-message by its msg-type
    switch (tmsg->msg_type) {
        case P9_TLOPEN:
            ret = p9_proto_tlopen(req, tmsg);
            break;
        case P9_TGETATTR:
            ret = p9_proto_tgetattr(req, tmsg);
            break;
        case P9_TVERSION:
            ret = p9_proto_tversion(req, tmsg);
            break;
        case P9_TATTACH:
            ret = p9_proto_tattach(req, tmsg);
            break;
        case P9_TWALK:
            ret = p9_proto_twalk(req, tmsg);
            break;
        case P9_TOPEN:
            ret = p9_proto_topen(req, tmsg);
            break;
        case P9_TREAD:
            ret = p9_proto_tread(req, tmsg);
            break;
        case P9_TWRITE:
            ret = p9_proto_twrite(req, tmsg);
            break;
        case P9_TCLUNK:
            ret = p9_proto_tclunk(req, tmsg);
            break;
        case P9_TREMOVE:
            ret = p9_proto_tremove(req, tmsg);
            break;
        case P9_TLCREATE:
            ret = p9_proto_tlcreate(req, tmsg);
            break;
        case P9_TREADDIR:
            ret = p9_proto_treaddir(req, tmsg);
            break;
        case P9_TMKDIR:
            ret = p9_proto_tmkdir(req, tmsg);
            break;
        default:
            LTRACEF("9p T-message type not supported: %u\n", tmsg->msg_type);
            ret = ERR_NOT_SUPPORTED;
            goto err;
    }

    if (ret != NO_ERROR) {
        LTRACEF("9p T-message (code: %u) failed: %d\n", tmsg->msg_type, ret);
        goto err;
    }

    if ((ret = p9_req_finalize(req)) != NO_ERROR) {
        goto err;
    }

    virtio_9p_req_send(p9dev, req);

    // wait for server's response
    if (event_wait_timeout(&req->io_event, VIRTIO_9P_RPC_TIMEOUT) != NO_ERROR) {
        ret = ERR_TIMED_OUT;
        goto err;
    }

    // read the message header from the returned request
    p9_req_receive(req, rmsg);

    // read the R-message according to its msg-type
    switch (rmsg->msg_type) {
        case P9_RLOPEN:
            ret = p9_proto_rlopen(req, rmsg);
            break;
        case P9_RGETATTR:
            ret = p9_proto_rgetattr(req, rmsg);
            break;
        case P9_RVERSION:
            ret = p9_proto_rversion(req, rmsg);
            break;
        case P9_RATTACH:
            ret = p9_proto_rattach(req, rmsg);
            break;
        case P9_RWALK:
            ret = p9_proto_rwalk(req, rmsg);
            break;
        case P9_ROPEN:
            ret = p9_proto_ropen(req, rmsg);
            break;
        case P9_RREAD:
            ret = p9_proto_rread(req, rmsg);
            break;
        case P9_RWRITE:
            ret = p9_proto_rwrite(req, rmsg);
            break;
        case P9_RCLUNK:
            ret = p9_proto_rclunk(req, rmsg);
            break;
        case P9_RREMOVE:
            ret = p9_proto_rremove(req, rmsg);
            break;
        case P9_RLERROR:
            ret = p9_proto_rlerror(req, rmsg);
            break;
        case P9_RLCREATE:
            ret = p9_proto_rlcreate(req, rmsg);
            break;
        case P9_RREADDIR:
            ret = p9_proto_rreaddir(req, rmsg);
            break;
        case P9_RMKDIR:
            ret = p9_proto_rmkdir(req, rmsg);
            break;
        default:
            LTRACEF("9p R-message type not supported: %u\n", tmsg->msg_type);
            ret = ERR_NOT_SUPPORTED;
            goto err;
    }

err:
    p9_req_release(req);

req_unlock:
    mutex_release(&p9dev->req_lock);

    return ret;
}

void virtio_9p_msg_destroy(virtio_9p_msg_t *msg)
{
    switch (msg->msg_type) {
        case P9_RVERSION:
            free(msg->msg.rversion.version);
            break;
        case P9_RREAD:
            free(msg->msg.rread.data);
            break;
        case P9_RREADDIR:
            free(msg->msg.rreaddir.data);
            break;
        default:
            // didn't allocate extra space in the message
            break;
    }
}
