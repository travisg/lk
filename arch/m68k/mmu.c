/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include "arch/m68k/mmu.h"

#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <lk/trace.h>

#define LOCAL_TRACE 1

#if M68K_MMU

#if M68K_MMU == 68040

// 68040's layout is
// 4 or 8K pages. only affects the bottom level
// 32 bit entries at all levels
//              L0,  L1,  L2
// bits:        7,   7,   6,  12 (4K pages)
// entries:     128, 128, 64
// bytes/table: 512, 512, 256
//
// if using 4K page tables for L1 and L2, and using a small root table (L0):
//                           L0,    L1,           L2
// entries:                  128,   8*128 (1024), 16*64 (1024)
// usable entries per level: 128/8, 1024/16,      1024

// 68040 L2 table entry
typedef struct pte {
    uint32_t page_address:20;
    uint32_t ur          :1;
    uint32_t g           :1;
    uint32_t u1          :1;
    uint32_t u0          :1;
    uint32_t s           :1;
    uint32_t cm          :2;
    uint32_t m           :1;
    uint32_t u           :1;
    uint32_t w           :1;
    uint32_t pdt         :2;
} pte_t;
static_assert(sizeof(pte_t) == 4, "");

// 68040 L1 table entry
typedef struct ptp {
    uint32_t table_address:24;
    uint32_t _1           :4;
    uint32_t u            :1;
    uint32_t w            :1;
    uint32_t udt          :2;
} ptp_t;
static_assert(sizeof(ptp_t) == 4, "");

// 68040 L0 table entry
typedef struct root_ptp {
    uint32_t table_address:23;
    uint32_t _1           :5;
    uint32_t u            :1;
    uint32_t w            :1;
    uint32_t udt          :2;
} root_ptp_t;
static_assert(sizeof(root_ptp_t) == 4, "");

// some constants based on this
#define L0_SHIFT_RAW 7
#define L1_SHIFT_RAW 7
#define L2_SHIFT_RAW 6

// number of entries to repeat per level to get our emulated tables
#define L0_REPEAT_SHIFT 3
#define L1_REPEAT_SHIFT 4

#define L0_REPEATS (1 << L0_REPEAT_SHIFT)
#define L1_REPEATS (1 << L1_REPEAT_SHIFT)
static_assert(L0_REPEATS == 8, "");
static_assert(L1_REPEATS == 16, "");

// number of entries per level
#define L0_ENTRIES_RAW (1 << L0_SHIFT_RAW)
#define L1_ENTRIES_RAW (1 << (L1_SHIFT_RAW + L0_REPEAT_SHIFT))
#define L2_ENTRIES_RAW (1 << (L2_SHIFT_RAW + L1_REPEAT_SHIFT))
static_assert(L0_ENTRIES_RAW == 128, "");
static_assert(L1_ENTRIES_RAW == 1024, "");
static_assert(L2_ENTRIES_RAW == 1024, "");

// number of bytes per level
#define L0_BYTES (L0_ENTRIES_RAW * sizeof(root_ptp_t))
#define L1_BYTES (L1_ENTRIES_RAW * sizeof(ptp_t))
#define L2_BYTES (L2_ENTRIES_RAW * sizeof(pte_t))
static_assert(L0_BYTES == 512, "");
static_assert(L1_BYTES == 4096, "");
static_assert(L2_BYTES == 4096, "");

// number of unique entries per level
// pow2 4+6+10
#define L0_ENTRIES (1u << (L0_SHIFT_RAW - L0_REPEAT_SHIFT))
#define L1_ENTRIES (1u << (L1_SHIFT_RAW - L1_REPEAT_SHIFT + L0_REPEAT_SHIFT))
#define L2_ENTRIES (1u << (L2_SHIFT_RAW + L1_REPEAT_SHIFT))
static_assert(L0_ENTRIES == 16, "");
static_assert(L1_ENTRIES == 64, "");
static_assert(L2_ENTRIES == 1024, "");

// for a given virtual address, which bits correspond to what layer of the table
#define L0_VADDR_SHIFT (32 - L0_SHIFT_RAW)
#define L1_VADDR_SHIFT (L0_VADDR_SHIFT - L1_SHIFT_RAW)
#define L2_VADDR_SHIFT (L1_VADDR_SHIFT - L2_SHIFT_RAW)

static_assert(L0_VADDR_SHIFT == 25, "");
static_assert(L1_VADDR_SHIFT == 18, "");
static_assert(L2_VADDR_SHIFT == 12, "");

static volatile root_ptp_t kernel_pgtable[L0_ENTRIES_RAW] __ALIGNED(L0_BYTES);
// XXX hack for now before we allocate from PMM
static ptp_t l1_table[L0_ENTRIES][L1_ENTRIES_RAW] __ALIGNED(L1_BYTES);
static pte_t l2_table[L0_ENTRIES*L1_ENTRIES][L2_ENTRIES_RAW] __ALIGNED(L2_BYTES);

#else
#error "unsupported m68k mmu"
#endif

static uint get_l0_index(vaddr_t vaddr) {
    return (vaddr >> (L0_VADDR_SHIFT + L0_REPEAT_SHIFT)) & (L0_ENTRIES-1);
}

static uint get_l1_index(vaddr_t vaddr) {
    return (vaddr >> (L1_VADDR_SHIFT + L1_REPEAT_SHIFT)) & (L1_ENTRIES-1);
}

static uint get_l2_index(vaddr_t vaddr) {
    return (vaddr >> L2_VADDR_SHIFT) & (L2_ENTRIES-1);
}

__NO_INLINE static void map_l0(volatile root_ptp_t *root_table, vaddr_t vaddr, paddr_t addr) {
    const unsigned int idx = get_l0_index(vaddr);
    LTRACEF("vaddr %#lx paddr %#lx, shifted idx: %u\n", vaddr, addr, idx);

    DEBUG_ASSERT(idx < L0_ENTRIES);

    for (unsigned int i = 0; i < L0_REPEATS; i++) {
        const paddr_t pa = addr + i * (1u << L1_SHIFT_RAW) * sizeof(ptp_t);
        const unsigned int offset_idx = (idx << L0_REPEAT_SHIFT) + i;

        DEBUG_ASSERT(offset_idx < L0_ENTRIES_RAW);

        const root_ptp_t ptp = {
            .table_address = pa >> 9,
            .u = 0, // not used
            .w = 0, // not write protected
            .udt = 3, // resident
        };
        root_table[offset_idx] = ptp;
        LTRACEF_LEVEL(1, "real addr: %lx, real idx: %d\n", pa, offset_idx);
    }
}

__NO_INLINE static void map_l1(volatile ptp_t *table, vaddr_t vaddr, paddr_t addr) {
    const unsigned int idx = get_l1_index(vaddr);
    LTRACEF("vaddr %#lx paddr %#lx, shifted idx: %u\n", vaddr, addr, idx);

    DEBUG_ASSERT(idx < L1_ENTRIES);

    for (unsigned int i = 0; i < L1_REPEATS; i++) {
        const paddr_t pa = addr + i * (1u << L2_SHIFT_RAW) * sizeof(pte_t);
        const unsigned int offset_idx = (idx << L1_REPEAT_SHIFT) + i;

        DEBUG_ASSERT(offset_idx < L1_ENTRIES_RAW);

        const ptp_t ptp = {
            .table_address = pa >> 8,
            .u = 0, // not used
            .w = 0, // not write protected
            .udt = 3, // resident
        };
        table[offset_idx] = ptp;
        LTRACEF_LEVEL(2, "real addr: %lx, real idx: %d\n", pa, offset_idx);
    }
}

__NO_INLINE static void map_l2(volatile pte_t *table, vaddr_t vaddr, paddr_t addr) {
    const unsigned int idx = get_l2_index(vaddr);
    LTRACEF_LEVEL(2, "vaddr %#lx paddr %#lx, shifted idx: %u\n", vaddr, addr, idx);

    DEBUG_ASSERT(idx < L2_ENTRIES);

    const pte_t pte = {
        .page_address = addr >> 12,
        .g = 0, // not global
        .s = 1, // supervisor
        .cm = 0, // cache mode, cacheable
        .m = 0, // not modified
        .u = 0, // not used
        .w = 0, // not write protected
        .pdt = 1, // resident
    };
    table[idx] = pte;
}

#define MMU_REG_ACCESSOR(reg) \
    static uint32_t get_##reg(void) { \
        uint32_t reg; \
        asm volatile("movec %%" #reg ", %0" : "=r"(reg) :: "memory"); \
        return reg; \
    } \
    static void set_##reg(uint32_t val) { \
        asm volatile("movec %0, %%" #reg :: "r"(val) : "memory"); \
    }

// Control register accessors
MMU_REG_ACCESSOR(tc);
MMU_REG_ACCESSOR(itt0);
MMU_REG_ACCESSOR(itt1);
MMU_REG_ACCESSOR(dtt0);
MMU_REG_ACCESSOR(dtt1);
MMU_REG_ACCESSOR(mmusr);
MMU_REG_ACCESSOR(urp);
MMU_REG_ACCESSOR(srp);

void m68k_mmu_early_init(void) {}

void m68k_mmu_init(void) {
    LTRACE_ENTRY;
    // for now, set up an identity map for the whole address space

    // allocate L1 tables and map into the L0 table
    vaddr_t va = 0;
    paddr_t pa = 0;
    const vaddr_t terminal_va = 0; // 16MB, XXX hack
    for (unsigned int i = 0; i < L0_ENTRIES; i++) {
        ptp_t * const l1 = &l1_table[i][0];
        LTRACEF("L1 table %d at %p\n", i, l1);

        map_l0(kernel_pgtable, va, (paddr_t)l1);

        // allocate L2 tables and map into the L1 table
        for (unsigned int j = 0; j < L1_ENTRIES; j++) {
            pte_t * const l2 = &l2_table[i*L1_ENTRIES + j][0];
            LTRACEF("L2 table %d at %p\n", j, l2);

            map_l1(l1, va, (paddr_t)l2);

            // for every L2 page table entry, map a page
            for (unsigned int k = 0; k < L2_ENTRIES; k++) {
                map_l2(l2, va, pa);
                va += 4096;
                pa += 4096;

                if (va == terminal_va) {
                    goto done;
                }
            }
        }
    }
done:
    // set the supervisor root pointer
    set_srp((uint32_t)(uintptr_t)kernel_pgtable);
    set_urp((uint32_t)(uintptr_t)kernel_pgtable);
    set_tc((1<<15)); // enable, 4K pages

    // Dump all the registers
    printf("TC %#x\n", get_tc());
    printf("ITT0 %#x\n", get_itt0());
    printf("ITT1 %#x\n", get_itt1());
    printf("DTT0 %#x\n", get_dtt0());
    printf("DTT1 %#x\n", get_dtt1());
    printf("MMUSR %#x\n", get_mmusr());
    printf("URP %#x\n", get_urp());
    printf("SRP %#x\n", get_srp());

    LTRACE_EXIT;
}

#endif // M68K_MMU