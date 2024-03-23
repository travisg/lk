/*
 * Copyright (c) 2009 Corey Tabaka
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <lk/compiler.h>

#ifndef ASSEMBLY

#include <arch/x86.h>

/* override of some routines */
static inline void arch_enable_ints(void) {
    CF;
    __asm__ volatile("sti");
}

static inline void arch_disable_ints(void) {
    __asm__ volatile("cli");
    CF;
}

static inline bool arch_ints_disabled(void) {
    x86_flags_t state;

    __asm__ volatile(
#if ARCH_X86_32
        "pushfl;"
        "popl %%eax"
#elif ARCH_X86_64
        "pushfq;"
        "popq %%rax"
#endif
        : "=a" (state)
        :: "memory");

    return !(state & (1<<9));
}

static inline ulong arch_cycle_count(void) {
#if X86_LEGACY
    return 0;
#else
    return __builtin_ia32_rdtsc();
#endif
}

/* use a global pointer to store the current_thread */
extern struct thread *_current_thread;

static inline struct thread *arch_get_current_thread(void) {
    return _current_thread;
}

static inline void arch_set_current_thread(struct thread *t) {
    _current_thread = t;
}

static inline uint arch_curr_cpu_num(void) {
    return 0;
}

#if ARCH_X86_64
// relies on SSE2
#define mb()        __asm__ volatile("mfence" : : : "memory")
#define rmb()       __asm__ volatile("lfence" : : : "memory")
#define wmb()       __asm__ volatile("sfence" : : : "memory")
#else
// Store to the top of the stack as a load/store barrier. Cannot
// rely on SS2 being intrinsically available for older i386 class hardware.
#define __storeload_barrier \
    __asm__ volatile("lock; addl $0, (%%esp)" : : : "memory", "cc")
#define mb()        __storeload_barrier
#define rmb()       __storeload_barrier
#define wmb()       __storeload_barrier
#endif

#ifdef WITH_SMP
// XXX probably too strict
#define smp_mb()    mb
#define smp_rmb()   rmb
#define smp_wmb()   wmb
#else
#define smp_mb()    CF
#define smp_wmb()   CF
#define smp_rmb()   CF
#endif

#endif // !ASSEMBLY
