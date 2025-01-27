/*
 * Copyright (c) 2015 Carlos Pizano-Uribe  cpu@chromium.org
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

/**
 * @file
 * @brief  Port object functions
 * @defgroup event Events
 *
 */

#include <kernel/port.h>

#include <kernel/thread.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/list.h>
#include <lk/pow2.h>
#include <malloc.h>
#include <string.h>

// write ports can be in two states, open and closed, which have a
// different magic number.

#define WRITEPORT_MAGIC_W (0x70727477) // 'prtw'
#define WRITEPORT_MAGIC_X (0x70727478) // 'prtx'

#define READPORT_MAGIC  (0x70727472)  // 'prtr'
#define PORTGROUP_MAGIC (0x70727467)  // 'prtg'
#define PORTHOLD_MAGIC (0x70727467)   // 'prth'

#define PORT_BUFF_SIZE      8
#define PORT_BUFF_SIZE_BIG 64

#define RESCHEDULE_POLICY 1

#define MAX_PORT_GROUP_COUNT 256

typedef struct {
    uint log2;
    uint avail;
    uint head;
    uint tail;
    port_packet_t packet[1];
} port_buf_t;

typedef struct {
    int magic;
    struct list_node node;
    port_buf_t *buf;
    struct list_node rp_list;
    port_mode_t mode;
    char name[PORT_NAME_LEN];
} write_port_t;

typedef struct {
    int magic;
    wait_queue_t wait;
    struct list_node rp_list;
} port_group_t;

typedef struct {
    int magic;
    struct list_node w_node;
    struct list_node g_node;
    port_buf_t *buf;
    void *ctx;
    wait_queue_t wait;
    write_port_t *wport;
    port_group_t *gport;
} read_port_t;


static struct list_node write_port_list;


static port_buf_t *make_buf(bool big) {
    uint pk_count = big ? PORT_BUFF_SIZE_BIG : PORT_BUFF_SIZE;
    uint size = sizeof(port_buf_t) + ((pk_count - 1) * sizeof(port_packet_t));
    port_buf_t *buf = (port_buf_t *) malloc(size);
    if (!buf)
        return NULL;
    buf->log2 = log2_uint(pk_count);
    buf->head = buf->tail = 0;
    buf->avail = pk_count;
    return buf;
}

static inline bool buf_is_empty(port_buf_t *buf) {
    return buf->avail == valpow2(buf->log2);
}

static status_t buf_write(port_buf_t *buf, const port_packet_t *packets, size_t count) {
    if (buf->avail < count)
        return ERR_NOT_ENOUGH_BUFFER;

    for (size_t ix = 0; ix != count; ix++) {
        buf->packet[buf->tail] = packets[ix];
        buf->tail = modpow2(++buf->tail, buf->log2);
    }
    buf->avail -= count;
    return NO_ERROR;
}

static status_t buf_read(port_buf_t *buf, port_result_t *pr) {
    if (buf_is_empty(buf))
        return ERR_NO_MSG;
    pr->packet = buf->packet[buf->head];
    buf->head = modpow2(++buf->head, buf->log2);
    ++buf->avail;
    return NO_ERROR;
}

// must be called before any use of ports.
void port_init(void) {
    list_initialize(&write_port_list);
}

status_t port_create(const char *name, port_mode_t mode, port_t *port) {
    if (!name || !port)
        return ERR_INVALID_ARGS;

    // only unicast ports can have a large buffer.
    if (mode & PORT_MODE_BROADCAST) {
        if (mode & PORT_MODE_BIG_BUFFER)
            return ERR_INVALID_ARGS;
    }

    if (strnlen(name, PORT_NAME_LEN) >= PORT_NAME_LEN)
        return ERR_INVALID_ARGS;


    // Add a stack-allocated port to the list until we can
    // replace it with a heap-allocated port.
    write_port_t stack_wp = { .magic = PORTHOLD_MAGIC };
    // We waste a few cycles here with a throwaway copy.
    strlcpy(stack_wp.name, name, sizeof(stack_wp.name));

    // lookup for existing port, return that if found.
    write_port_t *wp = NULL;
    THREAD_LOCK(state1);
    list_for_every_entry(&write_port_list, wp, write_port_t, node) {
        if (strcmp(wp->name, name) == 0) {
            // can't return closed or partial ports.
            if (wp->magic == WRITEPORT_MAGIC_X ||
                wp->magic == PORTHOLD_MAGIC)
                wp = NULL;
            THREAD_UNLOCK(state1);
            if (wp) {
                *port = (void *) wp;
                return ERR_ALREADY_EXISTS;
            } else {
                return ERR_BUSY;
            }
        }
    }
    list_add_tail(&write_port_list, &stack_wp.node);
    THREAD_UNLOCK(state1);

    // not found, create the write port and the circular buffer.
    wp = calloc(1, sizeof(write_port_t));
    if (!wp) {
        THREAD_LOCK(state2);
        list_delete(&stack_wp.node);
        THREAD_UNLOCK(state2);
        return ERR_NO_MEMORY;
    }

    wp->magic = WRITEPORT_MAGIC_W;
    wp->mode = mode;
    strlcpy(wp->name, name, sizeof(wp->name));
    list_initialize(&wp->rp_list);

    wp->buf = make_buf(mode & PORT_MODE_BIG_BUFFER);
    if (!wp->buf) {
        free(wp);
        THREAD_LOCK(state2);
        list_delete(&stack_wp.node);
        THREAD_UNLOCK(state2);
        return ERR_NO_MEMORY;
    }

    // Avoid a name collision by swapping the temporary placeholder out of the
    // list for the actual port.
    THREAD_LOCK(state2);
    // Let's reserve a stack allocated entry then swap it for the allocated one.
    list_add_tail(&write_port_list, &wp->node);
    list_delete(&stack_wp.node);
    THREAD_UNLOCK(state2);

    *port = (void *)wp;
    return NO_ERROR;
}

status_t port_open(const char *name, void *ctx, port_t *port) {
    if (!name || !port)
        return ERR_INVALID_ARGS;

    // assume success; create the read port and buffer now.
    read_port_t *rp = calloc(1, sizeof(read_port_t));
    if (!rp)
        return ERR_NO_MEMORY;

    rp->magic = READPORT_MAGIC;
    wait_queue_init(&rp->wait);
    rp->ctx = ctx;

    // |buf| might not be needed, but we always allocate outside the lock.
    // this buffer is only needed for broadcast ports, but we don't know
    // that here.
    port_buf_t *buf = make_buf(false);  // Small is enough.
    if (!buf) {
        free(rp);
        return ERR_NO_MEMORY;
    }

    // find the named write port and associate it with read port.
    status_t rc = ERR_NOT_FOUND;

    THREAD_LOCK(state);
    write_port_t *wp = NULL;
    list_for_every_entry(&write_port_list, wp, write_port_t, node) {
        if (strcmp(wp->name, name) == 0 &&
            wp->magic != PORTHOLD_MAGIC) {
            // found; add read port to write port list.
            rp->wport = wp;
            if (wp->buf) {
                // this is the first read port; transfer the circular buffer.
                list_add_tail(&wp->rp_list, &rp->w_node);
                rp->buf = wp->buf;
                wp->buf = NULL;
                rc = NO_ERROR;
            } else if (buf) {
                // not first read port.
                if (wp->mode & PORT_MODE_UNICAST) {
                    // cannot add a second listener.
                    rc = ERR_NOT_ALLOWED;
                    break;
                }
                // use the new (small) circular buffer.
                list_add_tail(&wp->rp_list, &rp->w_node);
                rp->buf = buf;
                buf = NULL;
                rc = NO_ERROR;
            } else {
                // |buf| allocation failed and the buffer was needed.
                rc = ERR_NO_MEMORY;
            }
            break;
        }
    }
    THREAD_UNLOCK(state);

    if (buf)
        free(buf);

    if (rc == NO_ERROR) {
        *port = (void *)rp;
    } else {
        free(rp);
    }
    return rc;
}

status_t port_group(port_t *ports, size_t count, port_t *group) {
    if (count > MAX_PORT_GROUP_COUNT)
        return ERR_TOO_BIG;

    // Allow empty port groups.
    if (count && !ports)
        return ERR_INVALID_ARGS;

    if (!group)
        return ERR_INVALID_ARGS;

    // assume success; create port group now.
    port_group_t *pg = calloc(1, sizeof(port_group_t));
    if (!pg)
        return ERR_NO_MEMORY;

    pg->magic = PORTGROUP_MAGIC;
    wait_queue_init(&pg->wait);
    list_initialize(&pg->rp_list);

    status_t rc = NO_ERROR;

    THREAD_LOCK(state);
    for (size_t ix = 0; ix != count; ix++) {
        read_port_t *rp = (read_port_t *)ports[ix];
        if ((rp->magic != READPORT_MAGIC) || rp->gport) {
            // wrong type of port, or port already part of a group,
            // in any case, undo the changes to the previous read ports.
            for (size_t jx = 0; jx != ix; jx++) {
                ((read_port_t *)ports[jx])->gport = NULL;
            }
            rc = ERR_BAD_HANDLE;
            break;
        }
        // link port group and read port.
        rp->gport = pg;
        list_add_tail(&pg->rp_list, &rp->g_node);
    }
    THREAD_UNLOCK(state);

    if (rc == NO_ERROR) {
        *group = (port_t *)pg;
    } else {
        free(pg);
    }
    return rc;
}

status_t port_group_add(port_t group, port_t port) {
    if (!port || !group)
        return ERR_INVALID_ARGS;

    // Make sure the user has actually passed in a port group and a read-port.
    port_group_t *pg = (port_group_t *)group;
    if (pg->magic != PORTGROUP_MAGIC)
        return ERR_INVALID_ARGS;

    read_port_t *rp = (read_port_t *)port;
    if (rp->magic != READPORT_MAGIC || rp->gport)
        return ERR_BAD_HANDLE;

    status_t rc = NO_ERROR;
    THREAD_LOCK(state);

    if (list_length(&pg->rp_list) == MAX_PORT_GROUP_COUNT) {
        rc = ERR_TOO_BIG;
    } else {
        rp->gport = pg;
        list_add_tail(&pg->rp_list, &rp->g_node);

        // If the new read port being added has messages available, try to wake
        // any readers that might be present.
        if (!buf_is_empty(rp->buf)) {
            wait_queue_wake_one(&pg->wait, false, NO_ERROR);
        }
    }

    THREAD_UNLOCK(state);

    return rc;
}

status_t port_group_remove(port_t group, port_t port) {
    if (!port || !group)
        return ERR_INVALID_ARGS;

    // Make sure the user has actually passed in a port group and a read-port.
    port_group_t *pg = (port_group_t *)group;
    if (pg->magic != PORTGROUP_MAGIC)
        return ERR_INVALID_ARGS;

    read_port_t *rp = (read_port_t *)port;
    if (rp->magic != READPORT_MAGIC || rp->gport != pg)
        return ERR_BAD_HANDLE;

    THREAD_LOCK(state);

    bool found = false;
    read_port_t *current_rp;
    list_for_every_entry(&pg->rp_list, current_rp, read_port_t, g_node) {
        if (current_rp == rp) {
            found = true;
        }
    }

    if (!found) {
        THREAD_UNLOCK(state);
        return ERR_BAD_HANDLE;
    }

    list_delete(&rp->g_node);

    THREAD_UNLOCK(state);

    return NO_ERROR;
}

status_t port_write(port_t port, const port_packet_t *pk, size_t count) {
    if (!port || !pk)
        return ERR_INVALID_ARGS;

    write_port_t *wp = (write_port_t *)port;
    THREAD_LOCK(state);
    if (wp->magic != WRITEPORT_MAGIC_W) {
        // wrong port type.
        THREAD_UNLOCK(state);
        return ERR_BAD_HANDLE;
    }

    status_t status = NO_ERROR;
    int awake_count = 0;

    if (wp->buf) {
        // there are no read ports, just write to the buffer.
        status = buf_write(wp->buf, pk, count);
    } else {
        // there are read ports. for each, write and attempt to wake a thread
        // from the port group or from the read port itself.
        read_port_t *rp;
        list_for_every_entry(&wp->rp_list, rp, read_port_t, w_node) {
            if (buf_write(rp->buf, pk, count) < 0) {
                // buffer full.
                status = ERR_PARTIAL_WRITE;
                continue;
            }

            int awaken = 0;
            if (rp->gport) {
                awaken = wait_queue_wake_one(&rp->gport->wait, false, NO_ERROR);
            }
            if (!awaken) {
                awaken = wait_queue_wake_one(&rp->wait, false, NO_ERROR);
            }

            awake_count += awaken;
        }
    }

    THREAD_UNLOCK(state);

#if RESCHEDULE_POLICY
    if (awake_count)
        thread_yield();
#endif

    return status;
}

static inline status_t read_no_lock(read_port_t *rp, lk_time_t timeout, port_result_t *result) {
    status_t status = buf_read(rp->buf, result);
    result->ctx = rp->ctx;

    if (status != ERR_NO_MSG)
        return status;

    // early return allows compiler to elide the rest for the group read case.
    if (!timeout)
        return ERR_TIMED_OUT;

    status_t wr = wait_queue_block(&rp->wait, timeout);
    if (wr != NO_ERROR)
        return wr;
    // recursive tail call is usually optimized away with a goto.
    return read_no_lock(rp, timeout, result);
}

status_t port_read(port_t port, lk_time_t timeout, port_result_t *result) {
    if (!port || !result)
        return ERR_INVALID_ARGS;

    status_t rc = ERR_GENERIC;
    read_port_t *rp = (read_port_t *)port;

    THREAD_LOCK(state);
    if (rp->magic == READPORT_MAGIC) {
        // dealing with a single port.
        rc = read_no_lock(rp, timeout, result);
    } else if (rp->magic == PORTGROUP_MAGIC) {
        // dealing with a port group.
        port_group_t *pg = (port_group_t *)port;
        do {
            // read each port with no timeout.
            // todo: this order is fixed, probably a bad thing.
            list_for_every_entry(&pg->rp_list, rp, read_port_t, g_node) {
                rc = read_no_lock(rp, 0, result);
                if (rc != ERR_TIMED_OUT)
                    goto read_exit;
            }
            // no data, block on the group waitqueue.
            rc = wait_queue_block(&pg->wait, timeout);
        } while (rc == NO_ERROR);
    } else {
        // wrong port type.
        rc = ERR_BAD_HANDLE;
    }

read_exit:
    THREAD_UNLOCK(state);
    return rc;
}

status_t port_destroy(port_t port) {
    if (!port)
        return ERR_INVALID_ARGS;

    write_port_t *wp = (write_port_t *) port;
    port_buf_t *buf = NULL;

    THREAD_LOCK(state);
    if (wp->magic != WRITEPORT_MAGIC_X) {
        // wrong port type.
        THREAD_UNLOCK(state);
        return ERR_BAD_HANDLE;
    }
    // remove self from global named ports list.
    list_delete(&wp->node);

    if (wp->buf) {
        // we have no readers.
        buf = wp->buf;
    } else {
        // for each reader:
        read_port_t *rp;
        list_for_every_entry(&wp->rp_list, rp, read_port_t, w_node) {
            // wake the read and group ports.
            wait_queue_wake_all(&rp->wait, false, ERR_CANCELLED);
            if (rp->gport) {
                wait_queue_wake_all(&rp->gport->wait, false, ERR_CANCELLED);
            }
            // remove self from reader ports.
            rp->wport = NULL;
        }
    }

    wp->magic = 0;
    THREAD_UNLOCK(state);

    free(buf);
    free(wp);
    return NO_ERROR;
}

status_t port_close(port_t port) {
    if (!port)
        return ERR_INVALID_ARGS;

    read_port_t *rp = (read_port_t *) port;
    port_buf_t *buf = NULL;

    THREAD_LOCK(state);
    if (rp->magic == READPORT_MAGIC) {
        // dealing with a read port.
        if (rp->wport) {
            // remove self from write port list and reassign the bufer if last.
            list_delete(&rp->w_node);
            if (list_is_empty(&rp->wport->rp_list)) {
                rp->wport->buf = rp->buf;
                rp->buf = NULL;
            } else {
                buf = rp->buf;
            }
        }
        if (rp->gport) {
            // remove self from port group list.
            list_delete(&rp->g_node);
        }
        // wake up waiters, the return code is ERR_OBJECT_DESTROYED.
        wait_queue_destroy(&rp->wait, true);
        rp->magic = 0;

    } else if (rp->magic == PORTGROUP_MAGIC) {
        // dealing with a port group.
        port_group_t *pg = (port_group_t *) port;
        // wake up waiters.
        wait_queue_destroy(&pg->wait, true);
        // remove self from reader ports.
        rp = NULL;
        list_for_every_entry(&pg->rp_list, rp, read_port_t, g_node) {
            rp->gport = NULL;
        }
        pg->magic = 0;

    } else if (rp->magic == WRITEPORT_MAGIC_W) {
        // dealing with a write port.
        write_port_t *wp = (write_port_t *) port;
        // mark it as closed. Now it can be read but not written to.
        wp->magic = WRITEPORT_MAGIC_X;
        THREAD_UNLOCK(state);
        return NO_ERROR;

    } else {
        THREAD_UNLOCK(state);
        return ERR_BAD_HANDLE;
    }

    THREAD_UNLOCK(state);

    free(buf);
    free(port);
    return NO_ERROR;
}

