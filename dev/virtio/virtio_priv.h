/*
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <stdint.h>

// status bits used by multiple transport busses
#define VIRTIO_STATUS_ACKNOWLEDGE (1<<0)
#define VIRTIO_STATUS_DRIVER      (1<<1)
#define VIRTIO_STATUS_DRIVER_OK   (1<<2)
#define VIRTIO_STATUS_FEATURES_OK (1<<3)
#define VIRTIO_STATUS_DEVICE_NEEDS_RESET (1<<6)
#define VIRTIO_STATUS_FAILED      (1<<7)

// device independent feature bits
#define VIRTIO_F_INDIRECT_DESC    (1<<28)
#define VIRTIO_F_EVENT_IDX        (1<<29)
#define VIRTIO_F_VERSION_1        (1ULL<<32)
#define VIRTIO_F_ACCESS_PLATFORM  (1ULL<<33)
#define VIRTIO_F_RING_PACKED      (1ULL<<34)
#define VIRTIO_F_IN_ORDER         (1ULL<<35)
#define VIRTIO_F_ORDER_PLATFORM   (1ULL<<36)
#define VIRTIO_F_SR_IOV           (1ULL<<37)
#define VIRTIO_F_NOTIFICATION_DATA (1ULL<<38)
#define VIRTIO_F_NOTIF_CONFIG_DATA (1ULL<<39)
#define VIRTIO_F_RING_RESET       (1ULL<<40)
#define VIRTIO_F_ADMIN_VQ         (1ULL<<41)

void virtio_dump_device_independent_features_bits(uint64_t feature);

