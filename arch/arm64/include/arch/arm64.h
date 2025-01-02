/*
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <stdbool.h>
#include <sys/types.h>
#include <lk/compiler.h>

__BEGIN_CDECLS

#define DSB __asm__ volatile("dsb sy" ::: "memory")
#define ISB __asm__ volatile("isb" ::: "memory")

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define ARM64_READ_SYSREG(reg) \
({ \
    uint64_t _val; \
    __asm__ volatile("mrs %0," TOSTRING(reg) : "=r" (_val)); \
    _val; \
})

#define ARM64_WRITE_SYSREG(reg, val) \
({ \
    __asm__ volatile("msr " TOSTRING(reg) ", %0" :: "r" (val)); \
    ISB; \
})

void arm64_context_switch(vaddr_t *old_sp, vaddr_t new_sp);

uint64_t arm64_get_boot_el(void);

/* exception handling */
struct arm64_iframe_long {
    uint64_t r[30];
    uint64_t lr;
    uint64_t usp;
    uint64_t elr;
    uint64_t spsr;
};

struct arm64_iframe_short {
    uint64_t r[18];
    uint64_t lr;
    uint64_t usp;
    uint64_t elr;
    uint64_t spsr;
};

struct arm64_stackframe {
    uint64_t fp;
    uint64_t pc;
};

struct thread;
extern void arm64_exception_base(void);
void arm64_el3_to_el1(void);
void arm64_fpu_exception(struct arm64_iframe_long *iframe);
void arm64_fpu_save_state(struct thread *thread);

static inline void arm64_fpu_pre_context_switch(struct thread *thread) {
    uint32_t cpacr = ARM64_READ_SYSREG(cpacr_el1);
    if ((cpacr >> 20) & 3) {
        arm64_fpu_save_state(thread);
        cpacr &= ~(3 << 20);
        ARM64_WRITE_SYSREG(cpacr_el1, cpacr);
    }
}

/* overridable syscall handler */
void arm64_syscall(struct arm64_iframe_long *iframe, bool is_64bit);

/* Local per-cpu cache flush routines.
 * These routines clean or invalidate the cache from the point of view
 * of a single cpu to the point of coherence.
 */
void arm64_local_invalidate_cache_all(void);
void arm64_local_clean_invalidate_cache_all(void);
void arm64_local_clean_cache_all(void);

/* Current Exception Level values, as contained in CurrentEL */
#define CurrentEL_EL1    (1 << 2)
#define CurrentEL_EL2    (2 << 2)

static inline bool arm64_is_kernel_in_hyp_mode(void) {
    return ARM64_READ_SYSREG(CURRENTEL) == CurrentEL_EL2;
}


__END_CDECLS

