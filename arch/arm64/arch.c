/*
 * Copyright (c) 2014-2016 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <lk/debug.h>
#include <stdlib.h>
#include <arch.h>
#include <arch/atomic.h>
#include <arch/ops.h>
#include <arch/arm64.h>
#include <arch/arm64/mmu.h>
#include <arch/mp.h>
#include <kernel/thread.h>
#include <lk/init.h>
#include <lk/main.h>
#include <platform.h>
#include <lk/trace.h>

#define LOCAL_TRACE 0

#if WITH_SMP
/* smp boot lock */
static spin_lock_t arm_boot_cpu_lock = 1;
static volatile int secondaries_to_init = 0;
#endif
static uint32_t cpu_in_irq_ctxt[SMP_MAX_CPUS];

enum handler_return arch_irq(struct arm64_iframe_short *frame) {
    enum handler_return ret;
    uint32_t cpu = arch_curr_cpu_num();
    DEBUG_ASSERT(cpu < SMP_MAX_CPUS);

    cpu_in_irq_ctxt[cpu] = 1;
    ret = platform_irq(frame);
    cpu_in_irq_ctxt[cpu] = 0;
    return ret;
}

bool arch_in_int_handler() {
    uint32_t cpu = arch_curr_cpu_num();
    DEBUG_ASSERT(cpu < SMP_MAX_CPUS);

    return (cpu_in_irq_ctxt[cpu] == 1);
}

static void arm64_cpu_early_init(void) {
    /* set the vector base */
    ARM64_WRITE_SYSREG(VBAR_EL1, (uint64_t)&arm64_exception_base);

    arch_enable_fiqs();
}

void arch_early_init(void) {
    arm64_cpu_early_init();
    platform_init_mmu_mappings();
}

void arch_stacktrace(uint64_t fp, uint64_t pc)
{
    struct arm64_stackframe frame;

    if (!fp) {
        frame.fp = (uint64_t)__builtin_frame_address(0);
        frame.pc = (uint64_t)arch_stacktrace;
    } else {
        frame.fp = fp;
        frame.pc = pc;
    }

    printf("stack trace:\n");
    while (frame.fp) {
        printf("0x%llx\n", frame.pc);

        /* Stack frame pointer should be 16 bytes aligned */
        if (frame.fp & 0xF)
            break;

        frame.pc = *((uint64_t *)(frame.fp + 8));
        frame.fp = *((uint64_t *)frame.fp);
    }
}

void arch_init(void) {
#if WITH_SMP
    arch_mp_init_percpu();

    LTRACEF("midr_el1 0x%llx\n", ARM64_READ_SYSREG(midr_el1));

    secondaries_to_init = SMP_MAX_CPUS - 1; /* TODO: get count from somewhere else, or add cpus as they boot */

    lk_init_secondary_cpus(secondaries_to_init);

    LTRACEF("releasing %d secondary cpus\n", secondaries_to_init);

    /* release the secondary cpus */
    spin_unlock(&arm_boot_cpu_lock);

    /* flush the release of the lock, since the secondary cpus are running without cache on */
    arch_clean_cache_range((addr_t)&arm_boot_cpu_lock, sizeof(arm_boot_cpu_lock));
#endif
}

void arch_quiesce(void) {
}

void arch_idle(void) {
    __asm__ volatile("wfi");
}

void arch_chain_load(void *entry, ulong arg0, ulong arg1, ulong arg2, ulong arg3) {
    PANIC_UNIMPLEMENTED;
}

/* switch to user mode, set the user stack pointer to user_stack_top, put the svc stack pointer to the top of the kernel stack */
void arch_enter_uspace(vaddr_t entry_point, vaddr_t user_stack_top) {
    DEBUG_ASSERT(IS_ALIGNED(user_stack_top, 16));

    thread_t *ct = get_current_thread();

    vaddr_t kernel_stack_top = (uintptr_t)ct->stack + ct->stack_size;
    kernel_stack_top = ROUNDDOWN(kernel_stack_top, 16);

    /* set up a default spsr to get into 64bit user space:
     * zeroed NZCV
     * no SS, no IL, no D
     * all interrupts enabled
     * mode 0: EL0t
     */
    uint32_t spsr = 0;

    arch_disable_ints();

    asm volatile(
        "mov    sp, %[kstack];"
        "msr    sp_el0, %[ustack];"
        "msr    elr_el1, %[entry];"
        "msr    spsr_el1, %[spsr];"
        "eret;"
        :
        : [ustack]"r"(user_stack_top),
        [kstack]"r"(kernel_stack_top),
        [entry]"r"(entry_point),
        [spsr]"r"(spsr)
        : "memory");
    __UNREACHABLE;
}

#if WITH_SMP
/* called from assembly */
void arm64_secondary_entry(ulong);
void arm64_secondary_entry(ulong asm_cpu_num) {
    uint cpu = arch_curr_cpu_num();
    if (cpu != asm_cpu_num)
        return;

    arm64_cpu_early_init();

    spin_lock(&arm_boot_cpu_lock);
    spin_unlock(&arm_boot_cpu_lock);

    /* run early secondary cpu init routines up to the threading level */
    lk_init_level(LK_INIT_FLAG_SECONDARY_CPUS, LK_INIT_LEVEL_EARLIEST, LK_INIT_LEVEL_THREADING - 1);

    arch_mp_init_percpu();

    LTRACEF("cpu num %d\n", cpu);

    /* we're done, tell the main cpu we're up */
    atomic_add(&secondaries_to_init, -1);
    __asm__ volatile("sev");

    lk_secondary_cpu_entry();
}
#endif

