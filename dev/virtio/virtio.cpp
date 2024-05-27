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

void virtio_dump_desc(const vring_desc &desc) {
    printf("vring descriptor %p\n", &desc);
    printf("\taddr  0x%llx\n", desc.addr);
    printf("\tlen   0x%x\n", desc.len);
    printf("\tflags 0x%hx\n", desc.flags);
    printf("\tnext  0x%hx\n", desc.next);
}

static void virtio_init(uint level) {
}

LK_INIT_HOOK(virtio, &virtio_init, LK_INIT_LEVEL_THREADING);
