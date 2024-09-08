/*
 * Copyright (c) 2021 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <arch/m68k.h>
#include <inttypes.h>
#include <lk/debug.h>
#include <lk/trace.h>
#include <assert.h>
#include <kernel/thread.h>
#include <target.h>

#define LOCAL_TRACE 0

typedef struct m68k_iframe {
    uint32_t d[8];
    uint32_t a[7];
    uint16_t sr;
    uint16_t pc_high;
    uint16_t pc_low;
    uint16_t format : 4;
    uint16_t vector_offset : 12;
} m68k_iframe_t;

typedef struct m68k_iframe_format_7 {
    m68k_iframe_t base;
    uint32_t effective_address;
    uint16_t ssw;
    uint16_t wb3_status;
    uint16_t wb2_status;
    uint16_t wb1_status;
    uint32_t fault_address;
    uint32_t wb3_address;
    uint32_t wb3_data;
    uint32_t wb2_address;
    uint32_t wb2_data;
    uint32_t wb1_address;
    uint32_t wb1_data;
    uint32_t push_data_1;
    uint32_t push_data_2;
    uint32_t push_data_3;
} m68k_iframe_format_7_t;

static_assert(sizeof(m68k_iframe_format_7_t) - sizeof(m68k_iframe_t) == (0x3c - 0x8), "");

void dump_iframe(const m68k_iframe_t *iframe) {
    printf("pc 0x%08x sr 0x%04x format %#x vector %#x\n", iframe->pc_low | iframe->pc_high << 16, iframe->sr,
            iframe->format, iframe->vector_offset / 4);
    printf("d0 0x%08x d1 0x%08x d2 0x%08x d3 0x%08x\n", iframe->d[0], iframe->d[1], iframe->d[2], iframe->d[3]);
    printf("d4 0x%08x d5 0x%08x d6 0x%08x d7 0x%08x\n", iframe->d[4], iframe->d[5], iframe->d[6], iframe->d[7]);
    printf("a0 0x%08x a1 0x%08x a2 0x%08x a3 0x%08x\n", iframe->a[0], iframe->a[1], iframe->a[2], iframe->a[3]);
    printf("a4 0x%08x a5 0x%08x a6 0x%08x\n", iframe->a[4], iframe->a[5], iframe->a[6]);
}

static void access_fault(m68k_iframe_t *frame) {
    printf("access fault\n");
    dump_iframe(frame);

    // dump additional frame 7 stuff
    m68k_iframe_format_7_t *f7 = (m68k_iframe_format_7_t *)frame;
    printf("effective address %#" PRIx32 "\n", f7->effective_address);
    printf("special status word %#" PRIx16 "\n", f7->ssw);
    printf("halting\n");
    for (;;);
}

void m68k_exception(m68k_iframe_t *frame) {
    uint8_t code = frame->vector_offset / 4;

    TRACEF("frame %p, code %#hhx\n", frame, code);

    switch (code ) {
        case 2:
            access_fault(frame);
            break;
    }

    dump_iframe(frame);

    printf("more stack:\n");
    hexdump8(frame, 256);

    panic("unimplemented exception %#hhx\n", code);
}

void m68k_trap_exception(m68k_iframe_t *frame) {
    panic("unhandled trap exception!\n");
}

void m68k_null_trap(m68k_iframe_t *frame) {
    panic("unhandled null trap exception!");
}

// defined in platform interrupt controller
extern enum handler_return m68k_platform_irq(uint8_t irq);

void m68k_irq(m68k_iframe_t *frame) {
    uint8_t code = frame->vector_offset / 4;

    LTRACEF("frame %p, code %#hhx\n", frame, code);

    THREAD_STATS_INC(interrupts);

    if (unlikely(code == 0)) {
        // spurious interrupt
        return;
    }

    target_set_debug_led(1, true);

    enum handler_return ret = m68k_platform_irq(code);

    target_set_debug_led(1, false);

    if (ret == INT_RESCHEDULE) {
        thread_preempt();
    }
}
