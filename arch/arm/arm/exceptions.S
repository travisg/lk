/*
 * Copyright (c) 2008-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <lk/asm.h>
#include <arch/asm.h>
#include <arch/arm/cores.h>

/* exception handling glue.
 * NOTE: only usable on armv6+ cores
 */

#define TIMESTAMP_IRQ 0

/* macros to align and unalign the stack on 8 byte boundary for ABI compliance */
.macro stack_align, tempreg
    /* make sure the stack is aligned */
    mov     \tempreg, sp
    tst     sp, #4
    subeq   sp, #4
    push    { \tempreg }

    /* tempreg holds the original stack */
.endm

.macro stack_restore, tempreg
    /* restore the potentially unaligned stack */
    pop     { \tempreg }
    mov     sp, \tempreg
.endm

/* save and disable the vfp unit */
.macro vfp_save, temp1
    /* save old fpexc */
    vmrs    \temp1, fpexc

    push    { \temp1 }

    /* hard disable the vfp unit */
    bic     \temp1, #(1<<30)
    vmsr    fpexc, \temp1
.endm

/* restore the vfp enable/disable state */
.macro vfp_restore, temp1
    /* restore fpexc */
    pop     { \temp1 }

    vmsr    fpexc, \temp1
.endm

/* Save callee trashed registers.
 * At exit r0 contains a pointer to the register frame.
 */
.macro save
    /* save spsr and r14 onto the svc stack */
    srsdb   #0x13!

    /* switch to svc mode, interrupts disabled */
    cpsid   i,#0x13

    /* save callee trashed regs and lr */
    push    { r0-r3, r12, lr }

    /* save user space sp/lr */
    sub     sp, #8
    stmia   sp, { r13, r14 }^

#if ARM_WITH_VFP
    /* save and disable the vfp unit */
    vfp_save    r0
#endif

    /* make sure the stack is 8 byte aligned */
    stack_align r0

    /* r0 now holds the pointer to the original iframe (before alignment) */
.endm

.macro save_offset, offset
    sub     lr, \offset
    save
.endm

.macro restore
    /* undo the stack alignment we did before */
    stack_restore r0

#if ARM_WITH_VFP
    /* restore the old state of the vfp unit */
    vfp_restore r0
#endif

    /* restore user space sp/lr */
    ldmia   sp, { r13, r14 }^
    add     sp, #8

    pop     { r0-r3, r12, lr }

    /* return to whence we came from */
    rfeia   sp!
.endm

/* Save all registers.
 * At exit r0 contains a pointer to the register frame.
 */
.macro saveall
    /* save spsr and r14 onto the svc stack */
    srsdb   #0x13!

    /* switch to svc mode, interrupts disabled */
    cpsid   i,#0x13

    /* save all regs */
    push    { r0-r12, lr }

    /* save user space sp/lr */
    sub     sp, #8
    stmia   sp, { r13, r14 }^

#if ARM_WITH_VFP
    /* save and disable the vfp unit */
    vfp_save    r0
#endif

    /* make sure the stack is 8 byte aligned */
    stack_align r0

    /* r0 now holds the pointer to the original iframe (before alignment) */
.endm

.macro saveall_offset, offset
    sub     lr, \offset
    saveall
.endm

.macro restoreall
    /* undo the stack alignment we did before */
    stack_restore r0

#if ARM_WITH_VFP
    /* restore the old state of the vfp unit */
    vfp_restore r0
#endif

    /* restore user space sp/lr */
    ldmia   sp, { r13, r14 }^
    add     sp, #8

    pop     { r0-r12, r14 }

    /* return to whence we came from */
    rfeia   sp!
.endm

FUNCTION(arm_undefined)
    save
    /* r0 now holds pointer to iframe */

    bl      arm_undefined_handler

    restore

#ifndef WITH_LIB_SYSCALL
FUNCTION(arm_syscall)
    saveall
    /* r0 now holds pointer to iframe */

    bl      arm_syscall_handler

    restoreall
#endif

FUNCTION(arm_prefetch_abort)
    saveall_offset #4
    /* r0 now holds pointer to iframe */

    bl      arm_prefetch_abort_handler

    restoreall

FUNCTION(arm_data_abort)
    saveall_offset #8
    /* r0 now holds pointer to iframe */

    bl      arm_data_abort_handler

    restoreall

FUNCTION(arm_reserved)
    b   .

FUNCTION(arm_irq)
#if TIMESTAMP_IRQ
    /* read the cycle count */
    mrc     p15, 0, sp, c9, c13, 0
    str     sp, [pc, #__irq_cycle_count - . - 8]
#endif

    save_offset    #4

    /* r0 now holds pointer to iframe */

    /* call into higher level code */
    bl arch_irq

    /* reschedule if the handler returns nonzero */
    cmp     r0, #0
    blne    thread_preempt

    restore

FUNCTION(arm_fiq)
    save_offset #4
    /* r0 now holds pointer to iframe */

    bl  platform_fiq

    restore

.ltorg

#if TIMESTAMP_IRQ
DATA(__irq_cycle_count)
    .word   0
#endif
