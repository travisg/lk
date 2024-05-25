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
#include <dev/virtio/virtio_ring.h>

__BEGIN_CDECLS

/* detect a virtio mmio hardware block
 * returns number of devices found */
int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride);

__END_CDECLS

