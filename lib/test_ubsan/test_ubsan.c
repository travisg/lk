// Copyright 2024, Google LLC

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <lk/console_cmd.h>
#include <lk/err.h>
#include <lk/utils.h>

#define TESTCASE_UBSAN(x) { .func = x, .name = #x }

typedef void (*test_ubsan_fp)(void);
struct test_ubsan_struct {
    test_ubsan_fp func;
    const char *const name;
};

static void add_overflow(void) {
    volatile int val = INT_MAX;
    volatile int val2 = 1;

    val += val2;
}

static void sub_overflow(void) {
    volatile int val = INT_MIN;
    volatile int val2 = 1;

    val -= val2;
}

static void mul_overflow(void) {
    volatile int val = INT_MAX / 2;
    volatile int val2 = 3;

    val *= val2;
}

static void negate_overflow(void) {
    volatile int val = INT_MIN;

    val = -val;
}

static void divrem_overflow(void) {
    volatile int val = 16;
    volatile int val2 = 0;

    val /= val2;
}

static void pointer_overflow(void) {
    volatile uintptr_t offset = UINTPTR_MAX;
    char buf[4];
    char *ptr = buf;

    ptr += offset;
    (void) ptr;
}

static void float_cast_overflow(void) {
    volatile uint32_t val;
    volatile uint32_t mul = 10;

    val =  mul * 0xFFFF * 1e9;
    (void) val;
}

static void shift_out_of_bounds_overflow(void) {
    volatile int pos = 1;
    volatile int val = INT_MAX;

    val <<= pos;
}

static void shift_out_of_bounds_negative(void) {
    volatile int neg = -1;
    volatile int val = 10;

    val <<= neg;
}

static void out_of_bounds_overflow(void) {
    volatile int val = 10;
    volatile int overflow = 4;
    volatile char above[4] = {};
    volatile int arr[4];
    volatile char below[4] = {};

    above[0] = below[0];
    arr[overflow] = val;
}

static void out_of_bounds_underflow(void) {
    volatile int val = 10;
    volatile int underflow = -1;
    volatile char above[4] = {};
    volatile int arr[4];
    volatile char below[4] = {};

    above[0] = below[0];
    arr[underflow] = val;
}

static void load_invalid_value_bool(void) {
    volatile char *dst, *src;
    bool val, val2;
    volatile char invalid_val = 2;

    src = &invalid_val;
    dst = (char *)&val;
    *dst = *src;

    val2 = val;
    (void) val2;
}

extern void load_invalid_value_enum(void);

static void nullptr_deref(void) {
    int *nullptr = NULL;
    int val = *nullptr;
    (void) val;
}

static void misaligned_access(void) {
    struct object {
        int val;
    };
    char buf[3];
    struct object *obj = (struct object *)buf;

    obj = (struct object*)(buf + 1);
    obj->val = 0;
}

static void object_size_mismatch(void) {
    struct object {
        int val;
    };

    char buf[3];
    struct object *obj = (struct object *)buf;

    obj->val = 0;
}

static void func(int a) { return; }
static void function_type_mismatch(void) {
    typedef void (*func_ptr)(void);
    func_ptr f = (func_ptr) func;
    f();
}

__attribute__((nonnull)) static void _nonnull_arg( void *ptr) { (void) ptr; }
static void nonnull_arg(void) {
    volatile void *nullptr = NULL;
    _nonnull_arg((void *)nullptr);
}

__attribute__((returns_nonnull)) static void* _nonnull_return(void) {
    volatile void *ret = NULL;
    return (void *)ret;
}

static void nonnull_return(void) {
    _nonnull_return();
}

static void vla_bound_not_positive(void) {
    volatile int size = 0;
    int arr[size];

    (void) arr;
}

static void *_alignment_assumption(void) __attribute__((__assume_aligned__(32)));
static void *_alignment_assumption(void) {
    volatile void *ptr = (void *)0x3;
    return (void *)ptr;
}

static void *_alignment_assumption_offset(void) __attribute__((__assume_aligned__(32, 4)));
static void *_alignment_assumption_offset(void) {
    volatile void *ptr = (void *)0x3;
    return (void *)ptr;
}

static void alignment_assumption(void) {
    void *ptr;

    ptr = _alignment_assumption();
    (void) ptr;
}

static void alignment_assumption_offset(void) {
    void *ptr;

    ptr = _alignment_assumption_offset();
    (void) ptr;
}

// __builtin_clzg and __builtin_ctzg is not support
static void invalid_builtin_clz(void) {
    int ret = __builtin_clz(0);
    (void) ret;
}

static void invalid_builtin_ctz(void) {
    int ret = __builtin_ctz(0);
    (void) ret;
}

static void builtin_unreachable(void) {
    __UNREACHABLE;
}

static const struct test_ubsan_struct test_ubsan_array[] = {
    TESTCASE_UBSAN(add_overflow),
    TESTCASE_UBSAN(sub_overflow),
    TESTCASE_UBSAN(mul_overflow),
    TESTCASE_UBSAN(negate_overflow),
    TESTCASE_UBSAN(pointer_overflow),
    TESTCASE_UBSAN(float_cast_overflow),
    TESTCASE_UBSAN(shift_out_of_bounds_overflow),
    TESTCASE_UBSAN(shift_out_of_bounds_negative),
    TESTCASE_UBSAN(out_of_bounds_overflow),
    TESTCASE_UBSAN(out_of_bounds_underflow),
    TESTCASE_UBSAN(load_invalid_value_bool),
    TESTCASE_UBSAN(load_invalid_value_enum),
    TESTCASE_UBSAN(nullptr_deref),
    TESTCASE_UBSAN(misaligned_access),
    TESTCASE_UBSAN(object_size_mismatch),
    TESTCASE_UBSAN(function_type_mismatch),
    TESTCASE_UBSAN(nonnull_arg),
    TESTCASE_UBSAN(nonnull_return),
    TESTCASE_UBSAN(vla_bound_not_positive),
    TESTCASE_UBSAN(alignment_assumption),
    TESTCASE_UBSAN(alignment_assumption_offset),
    TESTCASE_UBSAN(invalid_builtin_clz),
    TESTCASE_UBSAN(invalid_builtin_ctz),
    TESTCASE_UBSAN(builtin_unreachable),
};


static void show_usage(void) {
        printf("Usage: test_ubsan [OPTION]\n");
        printf("    NUM - The testcase number you want to test\n");
        printf("    all - executing all ubsan test cases\n");
        printf("Available ubsan test cases:\n");
        printf("Num: testcase name\n");
        for (unsigned int i = 0; i < ARRAY_SIZE(test_ubsan_array); ++i)
            printf("%3u: %s\n", i, test_ubsan_array[i].name);
}

static int test_ubsan(int argc, const console_cmd_args *argv) {

    unsigned long id;

    if (argc != 2) {
        show_usage();
        return  argc == 1 ? NO_ERROR : ERR_INVALID_ARGS;
    }

    if (!strcmp(argv[1].str, "all")) {
        for (unsigned int i = 0; i < ARRAY_SIZE(test_ubsan_array); ++i) {
            printf("start %s ...\n", test_ubsan_array[i].name);
            test_ubsan_array[i].func();
        }

        return NO_ERROR;
    }

    if (!isdigit(argv[1].str[0])) {
        show_usage();
        return ERR_INVALID_ARGS;
    }

    id = argv[1].u;

    if (id >= ARRAY_SIZE(test_ubsan_array)) {
        show_usage();
        return ERR_INVALID_ARGS;
    }

    printf("start %s ...\n", test_ubsan_array[id].name);
    test_ubsan_array[id].func();
    return NO_ERROR;
}

STATIC_COMMAND_START
STATIC_COMMAND("test_ubsan", "testing ubsan", test_ubsan)
STATIC_COMMAND_END(test_ubsan);
