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
#include "protocol.h"

#include <dev/virtio/9p.h>
#include <inttypes.h>
#include <lk/err.h>
#include <lk/trace.h>
#include <stdlib.h>

#define LOCAL_TRACE 0

/*
 * read/write of basic type APIs
 */

size_t pdu_read(struct p9_fcall *pdu, void *data, size_t size)
{
    size_t len = MIN(pdu->size - pdu->offset, size);

    memcpy(data, &pdu->sdata[pdu->offset], len);
    pdu->offset += len;
    return len;
}

size_t pdu_write(struct p9_fcall *pdu, void *data, size_t size)
{
    size_t len = MIN(pdu->capacity - pdu->offset, size);

    memcpy(&pdu->sdata[pdu->size], data, len);
    pdu->size += len;
    return len;
}

status_t pdu_writeb(struct p9_fcall *pdu, uint8_t byte)
{
    return pdu_write(pdu, &byte, 1) == 1 ? NO_ERROR : ERR_IO;
}

uint8_t pdu_readb(struct p9_fcall *pdu)
{
    uint8_t byte;
    ASSERT(pdu_read(pdu, &byte, 1) == 1);
    return byte;
}

status_t pdu_writew(struct p9_fcall *pdu, uint16_t word)
{
    word = LE16(word);
    return pdu_write(pdu, &word, 2) == 2 ? NO_ERROR : ERR_IO;
}

uint16_t pdu_readw(struct p9_fcall *pdu)
{
    uint16_t word;
    ASSERT(pdu_read(pdu, &word, 2) == 2);
    return LE16(word);
}

status_t pdu_writed(struct p9_fcall *pdu, uint32_t dword)
{
    dword = LE32(dword);
    return pdu_write(pdu, &dword, 4) == 4 ? NO_ERROR : ERR_IO;
}

uint32_t pdu_readd(struct p9_fcall *pdu)
{
    uint32_t dword;
    ASSERT(pdu_read(pdu, &dword, 4) == 4);
    return LE32(dword);
}

status_t pdu_writeq(struct p9_fcall *pdu, uint64_t qword)
{
    qword = LE64(qword);
    return pdu_write(pdu, &qword, 8) == 8 ? NO_ERROR : ERR_IO;
}

uint64_t pdu_readq(struct p9_fcall *pdu)
{
    uint64_t qword;
    ASSERT(pdu_read(pdu, &qword, 8) == 8);
    return LE64(qword);
}

status_t pdu_writestr(struct p9_fcall *pdu, const char *str)
{
    uint16_t len = strlen(str);
    status_t ret;

    if ((ret = pdu_writew(pdu, len)) != NO_ERROR)
        return ret;

    return pdu_write(pdu, (void *)str, len) == len ? NO_ERROR : ERR_IO;
}

char *pdu_readstr(struct p9_fcall *pdu)
{
    uint16_t len;
    char *str = NULL;

    len = pdu_readw(pdu);
    if (!len) {
        return NULL;
    }

    str = (char *)calloc(len + 1, sizeof(char));
    if (!str) {
        return NULL;
    }

    ASSERT(pdu_read(pdu, str, len) == len);
    return str;
}

virtio_9p_qid_t pdu_readqid(struct p9_fcall *pdu)
{
    virtio_9p_qid_t qid;
    qid.type = pdu_readb(pdu);
    qid.version = pdu_readd(pdu);
    qid.path = pdu_readq(pdu);
    return qid;
}

status_t pdu_writedata(struct p9_fcall *pdu, const uint8_t *data,
                       uint32_t count)
{
    status_t ret;
#if LOCAL_TRACE >= 2
    LTRACEF("count (%u) data (%p)\n", count, data);
    hexdump8(data, count);
#endif

    if ((ret = pdu_writed(pdu, count)) != NO_ERROR)
        return ret;

    return pdu_write(pdu, (void *)data, count) == count ? NO_ERROR : ERR_IO;
}

uint8_t *pdu_readdata(struct p9_fcall *pdu, uint32_t *count)
{
    uint8_t *data = NULL;

    *count = pdu_readd(pdu);
    if (*count == 0)
        return NULL;

    data = (uint8_t *)calloc(*count, sizeof(uint8_t));
    if (!data)
        return NULL;

    ASSERT(pdu_read(pdu, data, *count) == *count);
#if LOCAL_TRACE >= 2
    LTRACEF("count (%u) data (%p)\n", *count, data);
    hexdump8(data, *count);
#endif
    return data;
}

ssize_t p9_dirent_read(uint8_t *data, uint32_t size, p9_dirent_t *ent)
{
    struct p9_fcall fake_pdu;

    fake_pdu.sdata = data;
    fake_pdu.size = size;
    fake_pdu.capacity = size;
    fake_pdu.offset = 0;

    // Rreaddir pattern: qid[13] offset[8] type[1] name[s]
    ent->qid = pdu_readqid(&fake_pdu);
    ent->offset = pdu_readq(&fake_pdu);
    ent->type = pdu_readb(&fake_pdu);
    ent->name = pdu_readstr(&fake_pdu);

    LTRACEF(
        "9p read_dirent: qid.type (0x%x) qid.version (%u) qid.path (%llu) "
        "offset (%llu) type (0x%x) name (%s)\n",
        ent->qid.type, ent->qid.version, ent->qid.path, ent->offset, ent->type,
        ent->name);

    return fake_pdu.offset;
}

void p9_dirent_destroy(p9_dirent_t *ent)
{
    if (ent) {
        if (ent->name) {
            free(ent->name);
            ent->name = NULL;
        }
    }
}

/*
 * Plan 9 File Protocol APIs
 */

status_t p9_proto_tversion(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tversion pattern: msize[4] version[s]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tversion.msize)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writestr(&req->tc, tmsg->msg.tversion.version)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rversion(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rversion pattern: msize[4] version[s]
    rmsg->msg.rversion.msize = pdu_readd(&req->rc);
    rmsg->msg.rversion.version = pdu_readstr(&req->rc);

    LTRACEF("9p version: msize (%u), version (%s)\n", rmsg->msg.rversion.msize,
            rmsg->msg.rversion.version);

    return NO_ERROR;
}

status_t p9_proto_tattach(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tattach pattern: fid[4] afid[4] uname[s] aname[s] n_uname[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tattach.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tattach.afid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writestr(&req->tc, tmsg->msg.tattach.uname)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writestr(&req->tc, tmsg->msg.tattach.aname)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tattach.n_uname)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rattach(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rattach pattern: qid[13]
    rmsg->msg.rattach.qid = pdu_readqid(&req->rc);

    LTRACEF("9p attach: type (0x%x), version (%u), path (%llu)\n",
            rmsg->msg.rattach.qid.type, rmsg->msg.rattach.qid.version,
            rmsg->msg.rattach.qid.path);

    return NO_ERROR;
}

status_t p9_proto_twalk(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Twalk pattern: fid[4] newfid[4] nwname[2] nwname*(wname[s])
    if ((ret = pdu_writed(&req->tc, tmsg->msg.twalk.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.twalk.newfid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writew(&req->tc, tmsg->msg.twalk.nwname)) != NO_ERROR)
        return ret;
    for (int i = 0; i < tmsg->msg.twalk.nwname; i++) {
        if ((ret = pdu_writestr(&req->tc, tmsg->msg.twalk.wname[i])) != NO_ERROR)
            return ret;
    }

    return NO_ERROR;
}

status_t p9_proto_rwalk(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rwalk pattern: nwqid[2] nwqid*(qid[13])
    rmsg->msg.rwalk.nwqid = pdu_readw(&req->rc);

    for (int i = 0; i < rmsg->msg.rwalk.nwqid; i++) {
        rmsg->msg.rwalk.qid[i] = pdu_readqid(&req->rc);
        LTRACEF("9p walk: type (0x%x), version (%u), path (%llu)\n",
                rmsg->msg.rwalk.qid[i].type, rmsg->msg.rwalk.qid[i].version,
                rmsg->msg.rwalk.qid[i].path);
    }

    return NO_ERROR;
}

status_t p9_proto_topen(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Topen pattern: fid[4] mode[1]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.topen.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writeb(&req->tc, tmsg->msg.topen.mode)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_ropen(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Ropen pattern: qid[13] iounit[4]
    rmsg->msg.ropen.qid = pdu_readqid(&req->rc);
    rmsg->msg.ropen.iounit = pdu_readd(&req->rc);

    LTRACEF("9p open: type (0x%x), version (%u), path (%llu), iounit (%u)\n",
            rmsg->msg.ropen.qid.type, rmsg->msg.ropen.qid.version,
            rmsg->msg.ropen.qid.path, rmsg->msg.ropen.iounit);

    return NO_ERROR;
}

status_t p9_proto_tlopen(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tlopen pattern: fid[4] flags[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlopen.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlopen.flags)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rlopen(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rlopen pattern: qid[13] iounit[4]
    rmsg->msg.rlopen.qid = pdu_readqid(&req->rc);
    rmsg->msg.rlopen.iounit = pdu_readd(&req->rc);

    LTRACEF("9p lopen: type (0x%x), version (%u), path (%llu), iounit (%u)\n",
            rmsg->msg.rlopen.qid.type, rmsg->msg.rlopen.qid.version,
            rmsg->msg.rlopen.qid.path, rmsg->msg.rlopen.iounit);

    return NO_ERROR;
}

status_t p9_proto_tgetattr(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tgetattr pattern: fid[4] request_mask[8]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tgetattr.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writeq(&req->tc, tmsg->msg.tgetattr.request_mask)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rgetattr(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rgetattr pattern: valid[8] qid[13] mode[4] uid[4] gid[4] nlink[8]
    //                   rdev[8] size[8] blksize[8] blocks[8]
    //                   atime_sec[8] atime_nsec[8] mtime_sec[8] mtime_nsec[8]
    //                   ctime_sec[8] ctime_nsec[8] btime_sec[8] btime_nsec[8]
    //                   gen[8] data_version[8]

    rmsg->msg.rgetattr.valid = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.qid = pdu_readqid(&req->rc);
    rmsg->msg.rgetattr.mode = pdu_readd(&req->rc);
    rmsg->msg.rgetattr.uid = pdu_readd(&req->rc);
    rmsg->msg.rgetattr.gid = pdu_readd(&req->rc);
    rmsg->msg.rgetattr.nlink = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.rdev = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.size = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.blksize = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.blocks = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.atime_sec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.atime_nsec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.mtime_sec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.mtime_nsec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.ctime_sec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.ctime_nsec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.btime_sec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.btime_nsec = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.gen = pdu_readq(&req->rc);
    rmsg->msg.rgetattr.data_version = pdu_readq(&req->rc);

    LTRACEF(
        "9p getattr: valid (0x%llx), qid.type (0x%x), qid.version (%u), "
        "qid.path (%llu), mode (0x%x), uid (%u), gid (%u), nlink (%llu), rdev "
        "(%llu), size (%llu), blksize (%llu), blocks (%llu), atime_sec (%llu), "
        "atime_nsec (%llu), mtime_sec (%llu), mtime_nsec (%llu), ctime_sec "
        "(%llu), ctime_nsec (%llu), btime_sec (%llu), btime_nsec (%llu), gen "
        "(%llu), data_version (%llu)\n",
        rmsg->msg.rgetattr.valid, rmsg->msg.rgetattr.qid.type,
        rmsg->msg.rgetattr.qid.version, rmsg->msg.rgetattr.qid.path,
        rmsg->msg.rgetattr.mode, rmsg->msg.rgetattr.uid, rmsg->msg.rgetattr.gid,
        rmsg->msg.rgetattr.nlink, rmsg->msg.rgetattr.rdev,
        rmsg->msg.rgetattr.size, rmsg->msg.rgetattr.blksize,
        rmsg->msg.rgetattr.blocks, rmsg->msg.rgetattr.atime_sec,
        rmsg->msg.rgetattr.atime_nsec, rmsg->msg.rgetattr.mtime_sec,
        rmsg->msg.rgetattr.mtime_nsec, rmsg->msg.rgetattr.ctime_sec,
        rmsg->msg.rgetattr.ctime_nsec, rmsg->msg.rgetattr.btime_sec,
        rmsg->msg.rgetattr.btime_nsec, rmsg->msg.rgetattr.gen,
        rmsg->msg.rgetattr.data_version);

    return NO_ERROR;
}

status_t p9_proto_tread(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tread pattern: fid[4] offset[8] count[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tread.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writeq(&req->tc, tmsg->msg.tread.offset)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tread.count)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rread(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rread pattern: count[4] data[count]
    rmsg->msg.rread.data = pdu_readdata(&req->rc, &rmsg->msg.rread.count);

    LTRACEF("9p read: count (%u) data (%p)\n", rmsg->msg.rread.count, rmsg->msg.rread.data);

    return NO_ERROR;
}

status_t p9_proto_twrite(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Twrite pattern: fid[4] offset[8] count[4] data[count]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.twrite.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writeq(&req->tc, tmsg->msg.twrite.offset)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writedata(&req->tc, tmsg->msg.twrite.data, tmsg->msg.twrite.count)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rwrite(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rwrite pattern: count[4]
    rmsg->msg.rwrite.count = pdu_readd(&req->rc);

    LTRACEF("9p write: count %u\n", rmsg->msg.rwrite.count);

    return NO_ERROR;
}

status_t p9_proto_tclunk(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tclunk pattern: fid[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tclunk.fid)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rclunk(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rclunk pattern:

    return NO_ERROR;
}

status_t p9_proto_tremove(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tremove pattern: fid[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tremove.fid)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rremove(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rremove pattern:

    return NO_ERROR;
}

status_t p9_proto_rlerror(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rlerror pattern: ecode[4]
    rmsg->msg.rlerror.ecode = pdu_readd(&req->rc);

    LTRACEF("9p lerror: ecode %u\n", rmsg->msg.rlerror.ecode);

    return NO_ERROR;
}

status_t p9_proto_tlcreate(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tlcreate pattern: fid[4] name[s] flags[4] mode[4] gid[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlcreate.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writestr(&req->tc, tmsg->msg.tlcreate.name)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlcreate.flags)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlcreate.mode)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tlcreate.gid)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rlcreate(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rlcreate pattern: qid[13] iounit[4]
    rmsg->msg.rlcreate.qid = pdu_readqid(&req->rc);
    rmsg->msg.rlcreate.iounit = pdu_readd(&req->rc);

    LTRACEF("9p lcreate: type (0x%x), version (%u), path (%llu), iounit (%u)\n",
            rmsg->msg.rlcreate.qid.type, rmsg->msg.rlcreate.qid.version,
            rmsg->msg.rlcreate.qid.path, rmsg->msg.rlcreate.iounit);

    return NO_ERROR;
}

status_t p9_proto_treaddir(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Treaddir pattern: fid[4] offset[8] count[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.treaddir.fid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writeq(&req->tc, tmsg->msg.treaddir.offset)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.treaddir.count)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rreaddir(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rreaddir pattern: count[4] data[count]
    rmsg->msg.rreaddir.data = pdu_readdata(&req->rc, &rmsg->msg.rreaddir.count);

    LTRACEF("9p readdir: count (%u) data (%p)\n", rmsg->msg.rreaddir.count,
            rmsg->msg.rreaddir.data);

    return NO_ERROR;
}

status_t p9_proto_tmkdir(struct p9_req *req, const virtio_9p_msg_t *tmsg)
{
    status_t ret;

    // Tmkdir pattern: dfid[4] name[s] mode[4] gid[4]
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tmkdir.dfid)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writestr(&req->tc, tmsg->msg.tmkdir.name)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tmkdir.mode)) != NO_ERROR)
        return ret;
    if ((ret = pdu_writed(&req->tc, tmsg->msg.tmkdir.gid)) != NO_ERROR)
        return ret;

    return NO_ERROR;
}

status_t p9_proto_rmkdir(struct p9_req *req, virtio_9p_msg_t *rmsg)
{
    // Rmkdir pattern: qid[13]
    rmsg->msg.rmkdir.qid = pdu_readqid(&req->rc);

    LTRACEF("9p mkdir: type (0x%x), version (%u), path (%llu)\n",
            rmsg->msg.rmkdir.qid.type, rmsg->msg.rmkdir.qid.version,
            rmsg->msg.rmkdir.qid.path);

    return NO_ERROR;
}
