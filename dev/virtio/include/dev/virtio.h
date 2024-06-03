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

// Detect a virtio mmio hardware block.
// Returns number of devices found.
int virtio_mmio_detect(void *ptr, uint count, const uint irqs[], size_t stride);

// Scan pci bus for virtio devices. Only implemented if PCI is present.
// Returns number of devices found.
int virtio_pci_init(void);

__END_CDECLS

