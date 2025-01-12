/*
 * Copyright (c) 2007 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <string.h>
#include <stdlib.h>
#include <lk/debug.h>
#include <lk/trace.h>
#include <lk/err.h>
#include "ext2_priv.h"

#define LOCAL_TRACE 0

struct dircookie {
    ext2_t *fs;
    struct ext2_inode dir_inode;
    uint fileblock;
    uint cursor;
};

// TODO, move to ext2.c
static void ext2_dump_inode(struct ext2_inode *inode) {
#if LOCAL_TRACE
  printf("mode:   %d\n", inode->i_mode);
  printf("uid:    %d\n", inode->i_uid);
  printf("size:   %d\n", inode->i_size);
  printf("blocks: %d\n", inode->i_blocks);
  printf("flags:  0x%x\n", inode->i_flags);
#endif
}

/* read in the dir, look for the entry */
static int ext2_dir_lookup(ext2_t *ext2, struct ext2_inode *dir_inode, const char *name, inodenum_t *inum) {
    uint file_blocknum;
    int err;
    uint8_t *buf;
    size_t namelen = strlen(name);
#if LOCAL_TRACE
    printf("looking for %s in inode %p\n", name, dir_inode);
    ext2_dump_inode(dir_inode);
#endif

    if (!S_ISDIR(dir_inode->i_mode))
        return ERR_NOT_DIR;

    buf = malloc(EXT2_BLOCK_SIZE(ext2->sb));

    file_blocknum = 0;
    for (;;) {
        /* read in the offset */
        err = ext2_read_inode(ext2, dir_inode, buf, file_blocknum * EXT2_BLOCK_SIZE(ext2->sb), EXT2_BLOCK_SIZE(ext2->sb));
        if (err <= 0) {
            free(buf);
            return -1;
        }

        /* walk through the directory entries, looking for the one that matches */
        struct ext2_dir_entry_2 *ent;
        uint pos = 0;
        while (pos < EXT2_BLOCK_SIZE(ext2->sb)) {
            ent = (struct ext2_dir_entry_2 *)&buf[pos];

            LTRACEF("ent %d:%d: inode 0x%x, reclen %d, namelen %d, type: %d, name '%s'\n",
                    file_blocknum, pos, LE32(ent->inode), LE16(ent->rec_len), ent->name_len, ent->file_type , ent->name);

            /* sanity check the record length */
            if (LE16(ent->rec_len) == 0)
                break;

            if (ent->name_len == namelen && memcmp(name, ent->name, ent->name_len) == 0) {
                // match
                *inum = LE32(ent->inode);
                LTRACEF("match: inode %d\n", *inum);
                free(buf);
                return 1;
            }

            pos += ROUNDUP(LE16(ent->rec_len), 4);
        }

        file_blocknum++;

        /* sanity check the directory. 4MB should be enough */
        if (file_blocknum > 1024) {
            free(buf);
            return -1;
        }
    }
}

/* note, trashes path */
static int ext2_walk(ext2_t *ext2, char *path, struct ext2_inode *start_inode, inodenum_t *inum, int recurse) {
    char *ptr;
    struct ext2_inode inode;
    struct ext2_inode dir_inode;
    int err;
    bool done;

    LTRACEF("path '%s', start_inode %p, inum %p, recurse %d\n", path, start_inode, inum, recurse);
    ext2_dump_inode(start_inode);

    if (recurse > 4)
        return ERR_RECURSE_TOO_DEEP;

    /* chew up leading slashes */
    ptr = &path[0];
    while (*ptr == '/')
        ptr++;

    done = false;
    memcpy(&dir_inode, start_inode, sizeof(struct ext2_inode));
    while (!done) {
        /* process the first component */
        char *next_sep = strchr(ptr, '/');
        if (next_sep) {
            /* terminate the next component, giving us a substring */
            *next_sep = 0;
        } else {
            /* this is the last component */
            done = true;
        }

        LTRACEF("component '%s', done %d\n", ptr, done);

        /* do the lookup on this component */
        err = ext2_dir_lookup(ext2, &dir_inode, ptr, inum);
        if (err < 0)
            return err;

nextcomponent:
        LTRACEF("inum %u\n", *inum);

        /* load the next inode */
        err = ext2_load_inode(ext2, *inum, &inode);
        if (err < 0)
            return err;

        /* is it a symlink? */
        if (S_ISLNK(inode.i_mode)) {
            char link[512];

            LTRACEF("hit symlink\n");

            err = ext2_read_link(ext2, &inode, link, sizeof(link));
            if (err < 0)
                return err;

            LTRACEF("symlink read returns %d '%s'\n", err, link);

            /* recurse, parsing the link */
            if (link[0] == '/') {
                /* link starts with '/', so start over again at the rootfs */
                err = ext2_walk(ext2, link, &ext2->root_inode, inum, recurse + 1);
            } else {
                err = ext2_walk(ext2, link, &dir_inode, inum, recurse + 1);
            }

            LTRACEF("recursive walk returns %d\n", err);

            if (err < 0)
                return err;

            /* if we weren't done with our path parsing, start again with the result of this recurse */
            if (!done) {
                goto nextcomponent;
            }
        } else if (S_ISDIR(inode.i_mode)) {
            /* for the next cycle, point the dir inode at our new directory */
            memcpy(&dir_inode, &inode, sizeof(struct ext2_inode));
        } else {
            if (!done) {
                /* we aren't done and this walked over a nondir, abort */
                LTRACEF("not finished and component is nondir: i_mode: 0x%x\n", inode.i_mode);
                return ERR_NOT_FOUND;
            }
        }

        if (!done) {
            /* move to the next separator */
            ptr = next_sep + 1;

            /* consume multiple separators */
            while (*ptr == '/')
                ptr++;
        }
    }

    return 0;
}

/* do a path parse, looking up each component */
int ext2_lookup(ext2_t *ext2, const char *_path, inodenum_t *inum) {
    LTRACEF("path '%s', inum %p\n", _path, inum);

    char path[512];
    strlcpy(path, _path, sizeof(path));

    return ext2_walk(ext2, path, &ext2->root_inode, inum, 1);
}

status_t ext2_opendir(fscookie *cookie, const char *name, dircookie **dcookie) {
    LTRACEF("cookie %p name '%s' dircookie %p\n", cookie, name, dcookie);
    ext2_t *ext2 = (ext2_t *)cookie;
    int err;
    inodenum_t inum;
    //uint file_blocknum;
    if (name[0] == 0) {
      inum = EXT2_ROOT_INO;
    } else {
      err = ext2_lookup(ext2, name, &inum);
      if (err < 0) {
        LTRACEF("err = %d\n", err);
        return err;
      }
    }
    LTRACEF("inum = %d\n", inum);

    // allocate a dir cookie, point it at the first file, and stuff it in the dircookie jar
    dircookie *dir = calloc(1, sizeof(*dir));
    if (!dir) return ERR_NO_MEMORY;
    dir->fs = ext2;
    /* load the next inode */
    err = ext2_load_inode(ext2, inum, &dir->dir_inode);
    if (err < 0) {
        free(dir);
        return err;
    }

    if (!S_ISDIR(dir->dir_inode.i_mode)) {
        free(dir);
        return ERR_NOT_DIR;
    }

    *dcookie = dir;

    return 0;
}
status_t ext2_readdir(dircookie *cookie, struct dirent *ent_out) {
    int err;
    uint8_t *buf;
    struct ext2_dir_entry_2 *ent;
    uint pos;

    if (!ent_out)
        return ERR_INVALID_ARGS;

    LTRACEF("dircookie: %p, direnv: %p, %d:%d\n", cookie, ent_out, cookie->fileblock, cookie->cursor);

    if (cookie->fileblock >= cookie->dir_inode.i_blocks) {
      return ERR_NOT_FOUND;
    }

    // TODO, write a function that will get a reference to the block cache
    // then copy and byte-swap the ext2_dir_entry_2 in one action
    buf = malloc(EXT2_BLOCK_SIZE(cookie->fs->sb));

    err = ext2_read_inode(cookie->fs, &cookie->dir_inode, buf, cookie->fileblock * EXT2_BLOCK_SIZE(cookie->fs->sb), EXT2_BLOCK_SIZE(cookie->fs->sb));
    if (err <= 0) {
        free(buf);
        return -1;
    }

    pos = cookie->cursor;
    ent = (struct ext2_dir_entry_2 *)&buf[pos];

    LTRACEF("ent %d:%d: inode 0x%x, reclen %d, namelen %d\n", cookie->fileblock, pos, LE32(ent->inode), LE16(ent->rec_len), ent->name_len/* , ent->name*/);

    /* sanity check the record length */
    if (LE16(ent->rec_len) == 0) {
        free(buf);
        return -1;
    }

    pos += ROUNDUP(LE16(ent->rec_len), 4);
    if (ent->inode == 0) { // deleted file
      // TODO, assumes end of dir listing
      // it should continue to the next record, potentially in the next block
      free(buf);
      return ERR_NOT_FOUND;
    }
    if (pos >= EXT2_BLOCK_SIZE(cookie->fs->sb)) {
      cookie->fileblock++;
      cookie->cursor = 0;
    } else {
      cookie->cursor = pos;
    }
    int size = MIN(ent->name_len, FS_MAX_FILE_LEN-1);
    memcpy(ent_out->name, ent->name, size);
    ent_out->name[size] = 0;
    free(buf);
    return NO_ERROR;
}
status_t ext2_closedir(dircookie *cookie) {
  free(cookie);
  return NO_ERROR;
}
