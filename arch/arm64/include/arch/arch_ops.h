/*
 * Copyright (c) 2008-2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#ifndef ASSEMBLY

#include <stdbool.h>
#include <lk/compiler.h>
#include <lk/reg.h>
#include <arch/arm64.h>

__BEGIN_CDECLS

#define ENABLE_CYCLE_COUNTER 1

void arch_stacktrace(uint64_t fp, uint64_t pc);
enum handler_return platform_irq(struct arm64_iframe_short *frame);

// override of some routines
static inline void arch_enable_ints(void) {
    CF;
    __asm__ volatile("msr daifclr, #2" ::: "memory");
}

static inline void arch_disable_ints(void) {
    __asm__ volatile("msr daifset, #2" ::: "memory");
    CF;
}

static inline bool arch_ints_disabled(void) {
    unsigned long state;

    __asm__ volatile("mrs %0, daif" : "=r"(state));
    state &= (1<<7);

    return !!state;
}

static inline void arch_enable_fiqs(void) {
    CF;
    __asm__ volatile("msr daifclr, #1" ::: "memory");
}

static inline void arch_disable_fiqs(void) {
    __asm__ volatile("msr daifset, #1" ::: "memory");
    CF;
}

static inline bool arch_fiqs_disabled(void) {
    unsigned long state;

    __asm__ volatile("mrs %0, daif" : "=r"(state));
    state &= (1<<6);

    return !!state;
}

#define mb()        __asm__ volatile("dsb sy" : : : "memory")
#define rmb()       __asm__ volatile("dsb ld" : : : "memory")
#define wmb()       __asm__ volatile("dsb st" : : : "memory")

#ifdef WITH_SMP
#define smp_mb()    __asm__ volatile("dmb ish" : : : "memory")
#define smp_rmb()   __asm__ volatile("dmb ishld" : : : "memory")
#define smp_wmb()   __asm__ volatile("dmb ishst" : : : "memory")
#else
#define smp_mb()    CF
#define smp_wmb()   CF
#define smp_rmb()   CF
#endif

static inline ulong arch_cycle_count(void) {
//#warning no arch_cycle_count implementation
    return 0;
}

/* use the cpu local thread context pointer to store current_thread */
static inline struct thread *arch_get_current_thread(void) {
    return (struct thread *)ARM64_READ_SYSREG(tpidr_el1);
}

static inline void arch_set_current_thread(struct thread *t) {
    ARM64_WRITE_SYSREG(tpidr_el1, (uint64_t)t);
}

#if WITH_SMP
static inline uint arch_curr_cpu_num(void) {
    uint64_t mpidr =  ARM64_READ_SYSREG(mpidr_el1);
    return ((mpidr & ((1U << SMP_CPU_ID_BITS) - 1)) >> 8 << SMP_CPU_CLUSTER_SHIFT) | (mpidr & 0xff);
}
#else
static inline uint arch_curr_cpu_num(void) {
    return 0;
}
#endif

__END_CDECLS

#endif // ASSEMBLY

