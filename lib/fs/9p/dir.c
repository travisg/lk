/*
 * Copyright (c) 2024, Google Inc. All rights reserved.
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
#include <dev/virtio/9p.h>

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <lk/err.h>
#include <lk/trace.h>

#include "v9fs_priv.h"

#define LOCAL_TRACE 0

status_t v9fs_open_dir(fscookie *cookie, const char *path, dircookie **dcookie)
{
    v9fs_t *v9fs = (v9fs_t *)cookie;
    char temppath[FS_MAX_PATH_LEN];
    uint32_t flags;
    int ret;

    LTRACEF("v9fs (%p) path (%s) dcookie (%p)\n", v9fs, path, dcookie);

    strlcpy(temppath, path, sizeof(temppath));

    /* create the dir object */
    v9fs_dir_t *dir = calloc(1, sizeof(v9fs_dir_t));

    if (!dir) {
        ret = ERR_NO_MEMORY;
        goto err;
    }

    dir->fid.fid = get_unused_fid(v9fs);

    virtio_9p_msg_t twalk = {
        .msg_type = P9_TWALK,
        .tag = P9_TAG_DEFAULT,
        .msg.twalk = {
            .fid = v9fs->root.fid, .newfid = dir->fid.fid
        }
    };
    virtio_9p_msg_t rwalk = {};

    path_to_wname(temppath, &twalk.msg.twalk.nwname, twalk.msg.twalk.wname);

    LTRACEF("twalk.msg.twalk.nwname (%u)\n", twalk.msg.twalk.nwname);
    for (int i = 0; i < twalk.msg.twalk.nwname; i++) {
        LTRACEF("twalk.msg.twalk.wname[%d] (%s)\n", i, twalk.msg.twalk.wname[i]);

        if (strlen(twalk.msg.twalk.wname[i]) == 0) {
            twalk.msg.twalk.wname[i] = "/";
        }
    }

    if ((ret = virtio_9p_rpc(v9fs->dev, &twalk, &rwalk)) != NO_ERROR)
        goto err;

    if (rwalk.msg_type != P9_RWALK ||
            rwalk.msg.rwalk.nwqid != twalk.msg.twalk.nwname) {
        ret = ERR_NOT_FOUND;
        goto err;
    }

    // assume all dir are opened as readonly directory
    flags = O_DIRECTORY | O_RDONLY;

    virtio_9p_msg_t tlopen = {
        .msg_type= P9_TLOPEN,
        .tag = P9_TAG_DEFAULT,
        .msg.tlopen = {
            .fid = dir->fid.fid, .flags = flags,
        }
    };
    virtio_9p_msg_t rlopen = {};

    if ((ret = virtio_9p_rpc(v9fs->dev, &tlopen, &rlopen)) != NO_ERROR)
        goto des_rwalk;

    if (rlopen.msg_type != P9_RLOPEN) {
        ret = ERR_ACCESS_DENIED;
        goto des_rwalk;
    }

    if (rlopen.msg.rlopen.qid.type != P9_QTDIR) {
        ret = ERR_NOT_DIR;
        goto des_rwalk;
    }

    dir->v9fs = v9fs;
    dir->fid.qid = rlopen.msg.rlopen.qid;
    dir->fid.iounit = rlopen.msg.rlopen.iounit;
    dir->offset = 0;
    dir->head = 0;
    dir->tail = 0;

    *dcookie = (dircookie *)dir;
    list_add_tail(&v9fs->dirs, &dir->node);

    virtio_9p_msg_destroy(&rlopen);
    virtio_9p_msg_destroy(&rwalk);

    return NO_ERROR;

des_rwalk:
    virtio_9p_msg_destroy(&rwalk);
    put_fid(v9fs, dir->fid.fid);

err:
    free(dir);
    LTRACEF("failed: %d\n", ret);
    return ret;
}

status_t v9fs_mkdir(fscookie *cookie, const char *path)
{
    v9fs_t *v9fs = (v9fs_t *)cookie;
    char temppath[FS_MAX_PATH_LEN];
    char *dirname;
    uint32_t dfid, mode;
    int ret;

    LTRACEF("v9fs (%p) path (%s)\n", v9fs, path);

    strlcpy(temppath, path, sizeof(temppath));

    dfid = get_unused_fid(v9fs);

    virtio_9p_msg_t twalk = {
        .msg_type = P9_TWALK,
        .tag = P9_TAG_DEFAULT,
        .msg.twalk = {
            .fid = v9fs->root.fid, .newfid = dfid,
        }
    };
    virtio_9p_msg_t rwalk = {};

    // separate the directory and the dirname
    dirname = strrchr(temppath, '/');
    if (!dirname || dirname == temppath) {        // create on the root dir
        dirname = temppath;
        twalk.msg.twalk.nwname = 0;
    } else {                                      // create on a dir
        // parse the parent directory
        *dirname++ = '\0';
        path_to_wname(temppath, &twalk.msg.twalk.nwname, twalk.msg.twalk.wname);
    }

    // walk to the parent directory
    if ((ret = virtio_9p_rpc(v9fs->dev, &twalk, &rwalk)) != NO_ERROR)
        goto err;

    if (rwalk.msg_type != P9_RWALK ||
            rwalk.msg.rwalk.nwqid != twalk.msg.twalk.nwname) {
        ret = ERR_NOT_DIR;
        goto err;
    }

    // assume the dir is created as 040755
    mode = S_IFDIR | S_IRWXU |
           S_IRGRP | S_IWGRP |
           S_IROTH | S_IWOTH;

    virtio_9p_msg_t tmkdir = {
        .msg_type= P9_TMKDIR,
        .tag = P9_TAG_DEFAULT,
        .msg.tmkdir = {
            .dfid = dfid, .name = dirname, .mode = mode,
        }
    };
    virtio_9p_msg_t rmkdir = {};

    if ((ret = virtio_9p_rpc(v9fs->dev, &tmkdir, &rmkdir)) != NO_ERROR)
        goto des_rwalk;

    if (rmkdir.msg_type != P9_RMKDIR) {
        ret = ERR_NOT_ALLOWED;
    }

    virtio_9p_msg_destroy(&rmkdir);

des_rwalk:
    virtio_9p_msg_destroy(&rwalk);

err:
    put_fid(v9fs, dfid);
    return ret;
}

status_t v9fs_read_dir(dircookie *dcookie, struct dirent *ent)
{
    v9fs_dir_t *dir = (v9fs_dir_t *)dcookie;
    p9_dirent_t p9_dent;
    status_t ret = NO_ERROR;
    ssize_t sread = 0;

    LTRACEF("dir (%p) ent (%p)\n", dir, ent);

    if (!dir && !ent)
        return ERR_INVALID_ARGS;

    while (true) {
        // read directory entries from v9p to fill buffer
        if (dir->head == dir->tail) {
            virtio_9p_msg_t treaddir = {
                .msg_type= P9_TREADDIR,
                .tag = P9_TAG_DEFAULT,
                .msg.treaddir = {
                    .fid = dir->fid.fid, .offset = dir->offset,
                    .count = PAGE_SIZE,
                }
            };
            virtio_9p_msg_t rreaddir = {};

            if ((ret = virtio_9p_rpc(dir->v9fs->dev, &treaddir, &rreaddir)) !=
                    NO_ERROR) {
                return ret;
            }

            if (rreaddir.msg_type != P9_RREADDIR) {
                virtio_9p_msg_destroy(&rreaddir);
                return ERR_IO;
            }

            dir->head = 0;
            dir->tail = MIN(rreaddir.msg.rreaddir.count, PAGE_SIZE);
            memcpy(&dir->data, rreaddir.msg.rreaddir.data, dir->tail);
            LTRACEF("head (%u) tail (%u) data (%p)\n", dir->head, dir->tail,
                    dir->data);

            virtio_9p_msg_destroy(&rreaddir);

            if (dir->tail == 0) {
                // no more entries
                return ERR_OUT_OF_RANGE;
            }
        }

        // read the dirent from buffer
        while (dir->head < dir->tail) {
            sread = p9_dirent_read(dir->data + dir->head, dir->tail - dir->head,
                                   &p9_dent);
            if (sread < NO_ERROR) {
                // invalid dirent
                return sread;
            }

            LTRACEF(
                "head (%u) tail (%u) sread (%zd) offset (%llu) name "
                "(%s)\n",
                dir->head, dir->tail, sread, p9_dent.offset, p9_dent.name);

            dir->head += sread;
            snprintf(ent->name, FS_MAX_FILE_LEN, "%s", p9_dent.name);
            dir->offset = p9_dent.offset;

            p9_dirent_destroy(&p9_dent);

            return NO_ERROR;
        }
    }

    ASSERT(false); // Impossible to get here
}

status_t v9fs_close_dir(dircookie *dcookie)
{
    v9fs_dir_t *dir = (v9fs_dir_t *)dcookie;

    LTRACEF("dir (%p)\n", dir);

    if (!dir)
        return NO_ERROR;

    put_fid(dir->v9fs, dir->fid.fid);
    list_delete(&dir->node);
    free(dir);

    return NO_ERROR;
}
