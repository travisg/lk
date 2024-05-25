/*
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lk/compiler.h>
#include <sys/types.h>
#include <dev/virtio.h>
#include <dev/virtio/virtio-device.h>

status_t virtio_block_init(virtio_device *dev, uint32_t host_features) __NONNULL();

