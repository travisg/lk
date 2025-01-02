/*
 * Copyright (c) 2012-2019 LK Trusty Authors. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <stdint.h>
#include <platform/gic.h>
#include <lk/reg.h>

#if ARCH_ARM
#include <arch/arm.h>
#endif
#if ARCH_ARM64
#include <arch/arm64.h>
#endif

#ifdef ARCH_ARM
/*
 * AArch32 does not have 64 bit mmio support, but the gic spec allows 32 bit
 * upper and lower access to _most_ 64 bit gic registers (not GICR_VSGIPENDR,
 * GICR_VSGIR or GITS_SGIR).
 */
/* TODO: add mmio_read32 when needed */
static inline void mmio_write64(volatile uint64_t *ptr64, uint64_t val) {
    volatile uint32_t *ptr = (volatile uint32_t *)ptr64;
    mmio_write32(ptr, (uint32_t)val);
    mmio_write32(ptr + 1, val >> 32);
}
#endif

struct arm_gic {
    vaddr_t gicc_vaddr;
    size_t gicc_size;
    vaddr_t gicd_vaddr;
    size_t gicd_size;
    vaddr_t gicr_vaddr;
    size_t gicr_size;
};

#define NUM_ARM_GICS 1

extern struct arm_gic arm_gics[NUM_ARM_GICS];

#if GIC_VERSION > 2

#if WITH_LIB_SM
#define ARM_GIC_USE_DOORBELL_NS_IRQ 1
#define ARM_GIC_DOORBELL_IRQ 13
#endif

    /* GICv3/v4 */

#define GICV3_IRQ_GROUP_GRP0S   0
#define GICV3_IRQ_GROUP_GRP1NS  1
#define GICV3_IRQ_GROUP_GRP1S   2

#ifndef ARM_GIC_SELECTED_IRQ_GROUP
#define ARM_GIC_SELECTED_IRQ_GROUP GRP1S
#endif

#define COMBINE2(a, b)  a ## b
#define XCOMBINE2(a, b) COMBINE2(a,b)
#define GICV3_IRQ_GROUP XCOMBINE2(GICV3_IRQ_GROUP_, ARM_GIC_SELECTED_IRQ_GROUP)

/*
 * In ARMv8 for GICv3/v4, ARM suggest to use system register
 * to access GICC instead of memory map.
 */
#ifdef ARCH_ARM64

#define GICCREG_READ(gic, reg)  ARM64_READ_SYSREG(reg)
#define GICCREG_WRITE(gic, reg, val) ARM64_WRITE_SYSREG(reg, (uint64_t)val)

#else /* ARCH_ARM64 */

/* For 32bit mode, use different way to access registers */
#define GICCREG_READ(gic, reg) COMBINE2(arm_read_,reg)()
#define GICCREG_WRITE(gic, reg, val) COMBINE2(arm_write_,reg)(val)

GEN_CP15_REG_FUNCS(icc_ctlr_el1, 0, c12, c12, 4);
GEN_CP15_REG_FUNCS(icc_pmr_el1, 0, c4, c6, 0);
GEN_CP15_REG_FUNCS(icc_bpr0_el1, 0, c12, c8, 3);
GEN_CP15_REG_FUNCS(icc_iar0_el1, 0, c12, c8, 0);
GEN_CP15_REG_FUNCS(icc_eoir0_el1, 0, c12, c8, 1);
GEN_CP15_REG_FUNCS(icc_rpr_el1, 0, c12, c11, 3);
GEN_CP15_REG_FUNCS(icc_hppir0_el1, 0, c12, c8, 2);
GEN_CP15_REG_FUNCS(icc_bpr1_el1, 0, c12, c12, 3);
GEN_CP15_REG_FUNCS(icc_iar1_el1, 0, c12, c12, 0);
GEN_CP15_REG_FUNCS(icc_eoir1_el1, 0, c12, c12, 1);
GEN_CP15_REG_FUNCS(icc_hppir1_el1, 0, c12, c12, 2);
GEN_CP15_REG_FUNCS(icc_dir_el1, 0, c12, c11, 1);
GEN_CP15_REG_FUNCS(icc_sre_el1, 0, c12, c12, 5);
GEN_CP15_REG_FUNCS(icc_igrpen0_el1, 0, c12, c12, 6);
GEN_CP15_REG_FUNCS(icc_igrpen1_el1, 0, c12, c12, 7);
GEN_CP15_REG_FUNCS(icc_ap0r0_el1, 0, c12, c8, 4);
GEN_CP15_REG_FUNCS(icc_ap0r1_el1, 0, c12, c8, 5);
GEN_CP15_REG_FUNCS(icc_ap0r2_el1, 0, c12, c8, 6);
GEN_CP15_REG_FUNCS(icc_ap0r3_el1, 0, c12, c8, 7);
GEN_CP15_REG_FUNCS(icc_ap1r0_el1, 0, c12, c9, 0);
GEN_CP15_REG_FUNCS(icc_ap1r1_el1, 0, c12, c9, 1);
GEN_CP15_REG_FUNCS(icc_ap1r2_el1, 0, c12, c9, 2);
GEN_CP15_REG_FUNCS(icc_ap1r3_el1, 0, c12, c9, 3);
GEN_CP15_REG64_FUNCS(icc_sgi1r_el1, 0, c12);
GEN_CP15_REG64_FUNCS(icc_asgi1r_el1, 1, c12);
GEN_CP15_REG64_FUNCS(icc_sgi0r_el1, 2, c12);

#endif /* ARCH_ARM64 */

#if GICV3_IRQ_GROUP == GICV3_IRQ_GROUP_GRP0S
#define GICC_PRIMARY_HPPIR       icc_hppir0_el1
#define GICC_PRIMARY_IAR         icc_iar0_el1
#define GICC_PRIMARY_EOIR        icc_eoir0_el1
#define GICC_PRIMARY_SGIR        icc_sgi0r_el1
#else
#define GICC_PRIMARY_HPPIR       icc_hppir1_el1
#define GICC_PRIMARY_IAR         icc_iar1_el1
#define GICC_PRIMARY_EOIR        icc_eoir1_el1
#define GICC_PRIMARY_SGIR        icc_sgi1r_el1
#endif

#define GICC_LIMIT (0x0000)

#else /* GIC_VERSION > 2 */

#ifndef GICC_OFFSET
#define GICC_OFFSET (0x0000)
#endif

#define GICCREG_READ(gic, reg) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICC_OFFSET); \
        ASSERT(reg < GICC_LIMIT); \
        mmio_read32((volatile uint32_t *)(arm_gics[(gic)].gicc_vaddr + ((reg) - GICC_OFFSET))); \
    })
#define GICCREG_WRITE(gic, reg, val) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICC_OFFSET); \
        ASSERT(reg < GICC_LIMIT); \
        mmio_write32((volatile uint32_t *)(arm_gics[(gic)].gicc_vaddr + ((reg) - GICC_OFFSET)), (val)); \
    })
/* main cpu regs */
#define GICC_CTLR               (GICC_OFFSET + 0x0000)
#define GICC_PMR                (GICC_OFFSET + 0x0004)
#define GICC_BPR                (GICC_OFFSET + 0x0008)
#define GICC_IAR                (GICC_OFFSET + 0x000c)
#define GICC_EOIR               (GICC_OFFSET + 0x0010)
#define GICC_RPR                (GICC_OFFSET + 0x0014)
#define GICC_HPPIR              (GICC_OFFSET + 0x0018)
#define GICC_ABPR               (GICC_OFFSET + 0x001c)
#define GICC_AIAR               (GICC_OFFSET + 0x0020)
#define GICC_AEOIR              (GICC_OFFSET + 0x0024)
#define GICC_AHPPIR             (GICC_OFFSET + 0x0028)
#define GICC_APR(n)             (GICC_OFFSET + 0x00d0 + (n) * 4)
#define GICC_NSAPR(n)           (GICC_OFFSET + 0x00e0 + (n) * 4)
#define GICC_IIDR               (GICC_OFFSET + 0x00fc)
#if 0 /* GICC_DIR is not currently used by anything */
#define GICC_DIR                (GICC_OFFSET + 0x1000)
#endif
#define GICC_LIMIT              (GICC_OFFSET + 0x1000)
#define GICC_MIN_SIZE           (GICC_LIMIT - GICC_OFFSET)

#if WITH_LIB_SM
#define GICC_PRIMARY_HPPIR      GICC_AHPPIR
#define GICC_PRIMARY_IAR        GICC_AIAR
#define GICC_PRIMARY_EOIR       GICC_AEOIR
#else
#define GICC_PRIMARY_HPPIR      GICC_HPPIR
#define GICC_PRIMARY_IAR        GICC_IAR
#define GICC_PRIMARY_EOIR       GICC_EOIR
#endif

#endif /* GIC_VERSION > 2 */

#ifndef GICD_OFFSET
#define GICD_OFFSET (GICC_LIMIT)
#endif

#define GICDREG_READ(gic, reg) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICD_OFFSET); \
        ASSERT(reg < GICD_LIMIT); \
        mmio_read32((volatile uint32_t *)(arm_gics[(gic)].gicd_vaddr + ((reg) - GICD_OFFSET))); \
    })
#define GICDREG_WRITE(gic, reg, val) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICD_OFFSET); \
        ASSERT(reg < GICD_LIMIT); \
        mmio_write32((volatile uint32_t *)(arm_gics[(gic)].gicd_vaddr + ((reg) - GICD_OFFSET)), (val)); \
    })
/* distribution regs */
#define GICD_CTLR               (GICD_OFFSET + 0x000)
#define GICD_TYPER              (GICD_OFFSET + 0x004)
#define GICD_IIDR               (GICD_OFFSET + 0x008)
#define GICD_IGROUPR(n)         (GICD_OFFSET + 0x080 + (n) * 4)
#define GICD_ISENABLER(n)       (GICD_OFFSET + 0x100 + (n) * 4)
#define GICD_ICENABLER(n)       (GICD_OFFSET + 0x180 + (n) * 4)
#define GICD_ISPENDR(n)         (GICD_OFFSET + 0x200 + (n) * 4)
#define GICD_ICPENDR(n)         (GICD_OFFSET + 0x280 + (n) * 4)
#define GICD_ISACTIVER(n)       (GICD_OFFSET + 0x300 + (n) * 4)
#define GICD_ICACTIVER(n)       (GICD_OFFSET + 0x380 + (n) * 4)
#define GICD_IPRIORITYR(n)      (GICD_OFFSET + 0x400 + (n) * 4)
#define GICD_ITARGETSR(n)       (GICD_OFFSET + 0x800 + (n) * 4)
#define GICD_ICFGR(n)           (GICD_OFFSET + 0xc00 + (n) * 4)
#define GICD_NSACR(n)           (GICD_OFFSET + 0xe00 + (n) * 4)
#define GICD_SGIR               (GICD_OFFSET + 0xf00)
#define GICD_CPENDSGIR(n)       (GICD_OFFSET + 0xf10 + (n) * 4)
#define GICD_SPENDSGIR(n)       (GICD_OFFSET + 0xf20 + (n) * 4)
#if GIC_VERSION <= 2
/* for v3 and higher, these are defined later */
#define GICD_LIMIT              (GICD_OFFSET + 0x1000)
#define GICD_MIN_SIZE           (GICD_LIMIT - GICD_OFFSET)
#endif /* GIC_VERSION <= 2 */

/* GICD_CTRL  Register
 *            Non-Secure   Only_a_Single   two_Security
 * (1U << 8)  RES0         nASSGIreq       RES0
 * (1U << 7)  RES0         E1NWF           E1NWF
 * (1U << 5)  RES0         RES0            ARE_NS
 * (1U << 4)  ARE_NS       ARE             ARE_S
 * (1U << 2)  RES0         RES0            ENABLE_G1S
 * (1U << 1)  ENABLE_G1A   ENABLE_G1       ENABLE_G1NS
 * (1U << 0)  ENABLE_G1    ENABLE_G0       ENABLE_G0
 */
#define GICD_CTLR_RWP           (1U << 31)
#define GICD_CTLR_nASSGIreq     (1U << 8)
#define GICD_CTRL_E1NWF         (1U << 7)
#define GICD_CTLR_DS            (1U << 6)
#define GICD_CTLR_ARE_NS        (1U << 5)
#define GICD_CTLR_ARE_S         (1U << 4)
#define GICD_CTLR_ENABLE_G1S    (1U << 2)
#define GICD_CTLR_ENABLE_G1NS   (1U << 1)
#define GICD_CTLR_ENABLE_G0     (1U << 0)

#if GIC_VERSION > 2
/* some registers of GICD are 64 bit */
#define GICDREG_READ64(gic, reg) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICD_OFFSET); \
        ASSERT(reg < GICD_LIMIT); \
        mmio_read64((volatile uint64_t *)(arm_gics[(gic)].gicd_vaddr + ((reg) - GICD_OFFSET))); \
    })
#define GICDREG_WRITE64(gic, reg, val) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(reg >= GICD_OFFSET); \
        ASSERT(reg < GICD_LIMIT); \
        mmio_write64((volatile uint64_t *)(arm_gics[(gic)].gicd_vaddr + ((reg) - GICD_OFFSET)), (val)); \
    })

/* GICv3/v4 Distributor interface */
#define GICD_STATUSR            (GICD_OFFSET + 0x0010)
#define GICD_SETSPI_NSR         (GICD_OFFSET + 0x0040)
#define GICD_CLRSPI_NSR         (GICD_OFFSET + 0x0048)
#define GICD_SETSPI_SR          (GICD_OFFSET + 0x0050)
#define GICD_CLRSPI_SR          (GICD_OFFSET + 0x0058)
#define GICD_IGRPMODR(n)        (GICD_OFFSET + 0x0D00 + (n) * 4)
#define GICD_IROUTER(n)         (GICD_OFFSET + 0x6000 + (n) * 8)
#define GICD_LIMIT              (GICD_OFFSET + 0x10000)
#define GICD_MIN_SIZE           (GICD_LIMIT - GICD_OFFSET)

/* GICv3/v4 Redistrubutor interface */
#if GIC_VERSION == 3
#define GICR_CPU_OFFSET(cpu) ((cpu) * 0x20000)
#endif
#if GIC_VERSION == 4
#define GICR_CPU_OFFSET(cpu) ((cpu) * 0x40000)
#endif

#ifndef GICR_OFFSET
#define GICR_OFFSET (GICD_LIMIT)
#endif

#define GICRREG_READ(gic, cpu, reg) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(cpu < SMP_MAX_CPUS); \
        ASSERT(reg >= GICR_OFFSET); \
        ASSERT(reg < GICR_LIMIT); \
        mmio_read32((volatile uint32_t *)(arm_gics[(gic)].gicr_vaddr + GICR_CPU_OFFSET(cpu) + ((reg) - GICR_OFFSET))); \
    })
#define GICRREG_WRITE(gic, cpu, reg, val) ({ \
        ASSERT(gic < NUM_ARM_GICS); \
        ASSERT(cpu < SMP_MAX_CPUS); \
        ASSERT(reg >= GICR_OFFSET); \
        ASSERT(reg < GICR_LIMIT); \
        mmio_write32((volatile uint32_t *)(arm_gics[(gic)].gicr_vaddr + GICR_CPU_OFFSET(cpu) + ((reg) - GICR_OFFSET)), (val)); \
    })

#define GICR_CTRL               (GICR_OFFSET + 0x0000)
#define GICR_IIDR               (GICR_OFFSET + 0x0004)
#define GICR_TYPER              (GICR_OFFSET + 0x0008)
#define GICR_STATUSR            (GICR_OFFSET + 0x0010)
#define GICR_WAKER              (GICR_OFFSET + 0x0014)

/* The following GICR registers are on separate 64KB page */
#define GICR_SGI_OFFSET         (GICR_OFFSET + 0x10000)
#define GICR_IGROUPR0           (GICR_SGI_OFFSET + 0x0080)
#define GICR_ISENABLER0         (GICR_SGI_OFFSET + 0x0100)
#define GICR_ICENABLER0         (GICR_SGI_OFFSET + 0x0180)
#define GICR_ISPENDR0           (GICR_SGI_OFFSET + 0x0200)
#define GICR_ICPENDR0           (GICR_SGI_OFFSET + 0x0280)
#define GICR_ISACTIVER0         (GICR_SGI_OFFSET + 0x0300)
#define GICR_ICACTIVER0         (GICR_SGI_OFFSET + 0x0380)
#define GICR_IPRIORITYR(n)      (GICR_SGI_OFFSET + 0x0400 + (n) * 4)
#define GICR_ICFGR(n)           (GICR_SGI_OFFSET + 0x0C00 + (n) * 4)
#define GICR_IGRPMODR0          (GICR_SGI_OFFSET + 0x0D00)
#define GICR_NSACR              (GICR_SGI_OFFSET + 0x0E00)
#define GICR_LIMIT              (GICR_SGI_OFFSET + 0x1000)
#define GICR_MIN_SIZE           (GICR_LIMIT - GICR_OFFSET)
#endif /* GIC_VERSION > 2 */

// XXX: from trusty macros.h

#define ROUND_UP(n, d) (((n) + (size_t)(d) - 1) & ~((size_t)(d) - 1))
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
