/*
 * Copyright (c) 2014 Chris Anderson
 * Copyright (c) 2014 Brian Swetland
 * Copyright (c) 2015 Nathaniel Quillin
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

#include "minip-internal.h"

#include <lk/err.h>
#include <errno.h>
#include <iovec.h>
#include <lk/list.h>
#include <malloc.h>
#include <stdint.h>
#include <lk/trace.h>

#define LOCAL_TRACE 0

static struct list_node udp_list = LIST_INITIAL_VALUE(udp_list);

struct udp_listener {
    struct list_node list;
    uint16_t port;
    udp_callback_t callback;
    void *arg;
};

typedef struct udp_socket {
    uint32_t host;
    uint16_t sport;
    uint16_t dport;
    const uint8_t *mac;
} udp_socket_t;

typedef struct udp_hdr {
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t len;
    uint16_t chksum;
} __PACKED udp_hdr_t;


int udp_listen(uint16_t port, udp_callback_t cb, void *arg) {
    struct udp_listener *entry, *temp;

    list_for_every_entry_safe(&udp_list, entry, temp, struct udp_listener, list) {
        if (entry->port == port) {
            if (cb == NULL) {
                list_delete(&entry->list);
                return 0;
            }
            return -1;
        }
    }

    if ((entry = malloc(sizeof(struct udp_listener))) == NULL) {
        return -1;
    }

    entry->port = port;
    entry->callback = cb;
    entry->arg = arg;

    list_add_tail(&udp_list, &entry->list);

    return 0;
}

status_t udp_open(uint32_t host, uint16_t sport, uint16_t dport, udp_socket_t **handle) {
    LTRACEF("host %u.%u.%u.%u sport %u dport %u handle %p\n",
            IPV4_SPLIT(host), sport, dport, handle);
    udp_socket_t *socket;
    const uint8_t *dst_mac;

    if (handle == NULL) {
        return -EINVAL;
    }

    socket = (udp_socket_t *) malloc(sizeof(udp_socket_t));
    if (!socket) {
        return -ENOMEM;
    }

    dst_mac = arp_get_dest_mac(host);
    if (dst_mac == NULL) {
        free(socket);
        return -EHOSTUNREACH;
    }

    socket->host = host;
    socket->sport = sport;
    socket->dport = dport;
    socket->mac = dst_mac;

    *handle = socket;

    return NO_ERROR;
}

status_t udp_close(udp_socket_t *handle) {
    if (handle == NULL) {
        return -EINVAL;
    }

    free(handle);
    return NO_ERROR;
}

status_t udp_send_iovec(const iovec_t *iov, uint iov_count, udp_socket_t *handle) {
    pktbuf_t *p;
    struct eth_hdr *eth;
    struct ipv4_hdr *ip;
    udp_hdr_t *udp;
    status_t ret = NO_ERROR;
    void *buf;
    ssize_t len;

    if (handle == NULL || iov == NULL || iov_count == 0) {
        return -EINVAL;
    }

    if ((p = pktbuf_alloc()) == NULL) {
        return -ENOMEM;
    }

    len = iovec_size(iov, iov_count);

    buf = pktbuf_append(p, len);
    udp = pktbuf_prepend(p, sizeof(udp_hdr_t));
    ip = pktbuf_prepend(p, sizeof(struct ipv4_hdr));
    eth = pktbuf_prepend(p, sizeof(struct eth_hdr));

    iovec_to_membuf(buf, len, iov, iov_count, 0);

    udp->src_port   = htons(handle->sport);
    udp->dst_port   = htons(handle->dport);
    udp->len        = htons(sizeof(udp_hdr_t) + len);
    udp->chksum     = 0;

    minip_build_mac_hdr(eth, handle->mac, ETH_TYPE_IPV4);
    minip_build_ipv4_hdr(ip, handle->host, IP_PROTO_UDP, len + sizeof(udp_hdr_t));

#if (MINIP_USE_UDP_CHECKSUM != 0)
    udp->chksum = rfc768_chksum(ip, udp);
#endif

    LTRACEF("packet paylod len %zd\n", len);

    minip_tx_handler(minip_tx_arg, p);

    return ret;
}

status_t udp_send(void *buf, size_t len, udp_socket_t *handle) {
    iovec_t iov;

    LTRACEF("buf %p, len %zu, handle %p\n", buf, len, handle);

    if (buf == NULL || len == 0) {
        return -EINVAL;
    }

    iov.iov_base = buf;
    iov.iov_len = len;

    return udp_send_iovec(&iov, 1, handle);
}

void udp_input(pktbuf_t *p, uint32_t src_ip) {
    udp_hdr_t *udp;
    struct udp_listener *e;
    uint16_t port;

    if ((udp = pktbuf_consume(p, sizeof(udp_hdr_t))) == NULL) {
        return;
    }

    port = ntohs(udp->dst_port);

    list_for_every_entry(&udp_list, e, struct udp_listener, list) {
        if (e->port == port) {
            e->callback(p->data, p->dlen, src_ip, ntohs(udp->src_port), e->arg);
            return;
        }
    }
}
