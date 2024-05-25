/*
 * Copyright (c) 2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lk/compiler.h>
#include <sys/types.h>
#include <dev/virtio.h>

__BEGIN_CDECLS

struct virtio_device;

status_t virtio_net_init(struct virtio_device *dev) __NONNULL();
status_t virtio_net_start(void);

/* return the count of virtio interfaces found */
int virtio_net_found(void);

status_t virtio_net_get_mac_addr(uint8_t mac_addr[6]);

struct pktbuf;
extern status_t virtio_net_send_minip_pkt(void *arg, struct pktbuf *p);

__END_CDECLS
