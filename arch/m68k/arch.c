/*
 * Copyright (c) 2021 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <lk/trace.h>
#include <lk/debug.h>
#include <stdint.h>
#include <arch/ops.h>
#include <arch/m68k.h>
#include <arch/m68k/mmu.h>
#include <kernel/spinlock.h>

#define LOCAL_TRACE 0

void arch_early_init(void) {
    LTRACE;

    arch_disable_ints();

#if M68K_CPU >= 68010
    // set the exception vector base
    extern uint32_t exc_vectors[256];
    asm volatile("movec %0, %%vbr" :: "r"(exc_vectors));
#endif
#if M68K_MMU
    m68k_mmu_early_init();
#endif
}

void arch_init(void) {
    LTRACE;
#if M68K_MMU
    m68k_mmu_init();
#endif
}

void arch_idle(void) {
    // set the SR such that we're in supervisor state and no ints are masked
    asm("stop #0x2000" ::: "cc");
}

void arch_chain_load(void *entry, ulong arg0, ulong arg1, ulong arg2, ulong arg3) {
    PANIC_UNIMPLEMENTED;
}

/* unimplemented cache operations */
void arch_disable_cache(uint flags) { PANIC_UNIMPLEMENTED; }
void arch_enable_cache(uint flags) { PANIC_UNIMPLEMENTED; }

void arch_clean_cache_range(addr_t start, size_t len) { PANIC_UNIMPLEMENTED; }
void arch_clean_invalidate_cache_range(addr_t start, size_t len) { PANIC_UNIMPLEMENTED; }
void arch_invalidate_cache_range(addr_t start, size_t len) { PANIC_UNIMPLEMENTED; }
void arch_sync_cache_range(addr_t start, size_t len) { PANIC_UNIMPLEMENTED; }

/* atomics that may need to be implemented */
// from https://gcc.gnu.org/wiki/Atomic/GCCMM/LIbrary
unsigned int  __atomic_fetch_add_4  (volatile void *mem, unsigned int val, int model) {
    spin_lock_saved_state_t state;
    arch_interrupt_save(&state, 0);
    unsigned int old = *(volatile unsigned int *)mem;
    *(volatile unsigned int *)mem = old + val;
    arch_interrupt_restore(state, 0);
    return old;
}
