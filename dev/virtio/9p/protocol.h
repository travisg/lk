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

#include <dev/virtio/9p.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <lk/list.h>
#include <sys/types.h>
#include <string.h>

#define VIRTIO_9P_RPC_TIMEOUT 3000 /* ms */
#define VIRTIO_9P_DEFAULT_MSIZE (PAGE_SIZE << 5)

struct p9_fcall {
    uint32_t size;

    size_t offset;
    size_t capacity;

    uint8_t *sdata;
};

struct p9_req {
    int status;
    event_t io_event;
    struct p9_fcall tc;
    struct p9_fcall rc;
};

enum {
    P9_REQ_S_UNKNOWN = 0,
    P9_REQ_S_INITIALIZED,
    P9_REQ_S_SENT,
    P9_REQ_S_RECEIVED,
};

struct virtio_9p_dev {
    struct virtio_device *dev;
    const struct virtio_9p_config *config;
    bdev_t bdev;

    uint32_t msize;
    struct p9_req req;
    mutex_t req_lock;

    struct list_node list;
    spin_lock_t lock;
};

// read/write APIs of basic types
size_t pdu_read(struct p9_fcall *pdu, void *data, size_t size);
size_t pdu_write(struct p9_fcall *pdu, void *data, size_t size);
status_t pdu_writeb(struct p9_fcall *pdu, uint8_t byte);
uint8_t pdu_readb(struct p9_fcall *pdu);
status_t pdu_writew(struct p9_fcall *pdu, uint16_t word);
uint16_t pdu_readw(struct p9_fcall *pdu);
status_t pdu_writed(struct p9_fcall *pdu, uint32_t dword);
uint32_t pdu_readd(struct p9_fcall *pdu);
status_t pdu_writeq(struct p9_fcall *pdu, uint64_t qword);
uint64_t pdu_readq(struct p9_fcall *pdu);
status_t pdu_writestr(struct p9_fcall *pdu, const char *str);
char *pdu_readstr(struct p9_fcall *pdu);
virtio_9p_qid_t pdu_readqid(struct p9_fcall *pdu);
status_t pdu_writedata(struct p9_fcall *pdu, const uint8_t *data, uint32_t count);
uint8_t *pdu_readdata(struct p9_fcall *pdu, uint32_t *count);

// Plan 9 File Protocol APIs
status_t p9_proto_tversion(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rversion(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tattach(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rattach(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_twalk(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rwalk(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_topen(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_ropen(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tlopen(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rlopen(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tgetattr(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rgetattr(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tread(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rread(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_twrite(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rwrite(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tclunk(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rclunk(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tremove(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rremove(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_rlerror(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tlcreate(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rlcreate(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_treaddir(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rreaddir(struct p9_req *req, virtio_9p_msg_t *rmsg);
status_t p9_proto_tmkdir(struct p9_req *req, const virtio_9p_msg_t *tmsg);
status_t p9_proto_rmkdir(struct p9_req *req, virtio_9p_msg_t *rmsg);
