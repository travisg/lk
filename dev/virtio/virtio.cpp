/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio.h>
#include <dev/virtio/virtio_ring.h>

#include <lk/debug.h>
#include <lk/init.h>
#include <stdio.h>
#include <inttypes.h>

#include "virtio_priv.h"

void virtio_dump_desc(const vring_desc &desc) {
    printf("vring descriptor %p\n", &desc);
    printf("\taddr  0x%llx\n", desc.addr);
    printf("\tlen   0x%x\n", desc.len);
    printf("\tflags 0x%hx\n", desc.flags);
    printf("\tnext  0x%hx\n", desc.next);
}

void virtio_dump_device_independent_features_bits(uint64_t feature) {
    printf("virtio device independent features (%#" PRIx64 "):", feature);
    if (feature & VIRTIO_F_INDIRECT_DESC) printf(" INDIRECT_DESC");
    if (feature & VIRTIO_F_EVENT_IDX) printf(" EVENT_IDX");
    if (feature & VIRTIO_F_VERSION_1) printf(" VERSION_1");
    if (feature & VIRTIO_F_ACCESS_PLATFORM) printf(" ACCESS_PLATFORM");
    if (feature & VIRTIO_F_RING_PACKED) printf(" RING_PACKED");
    if (feature & VIRTIO_F_IN_ORDER) printf(" IN_ORDER");
    if (feature & VIRTIO_F_ORDER_PLATFORM) printf(" ORDER_PLATFORM");
    if (feature & VIRTIO_F_SR_IOV) printf(" SR_IOV");
    if (feature & VIRTIO_F_NOTIFICATION_DATA) printf(" NOTIFICATION_DATA");
    if (feature & VIRTIO_F_NOTIF_CONFIG_DATA) printf(" NOTIF_CONFIG_DATA");
    if (feature & VIRTIO_F_RING_RESET) printf(" RING_RESET");
    if (feature & VIRTIO_F_ADMIN_VQ) printf(" ADMIN_VQ");
    printf("\n");
}

static void virtio_init(uint level) {
}

LK_INIT_HOOK(virtio, &virtio_init, LK_INIT_LEVEL_THREADING);
