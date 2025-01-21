/*
 * Copyright (c) 2008-2012 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <limits.h>
#include <stdint.h>
#include <stddef.h>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;

typedef long long     off_t;

typedef int status_t;

typedef uintptr_t addr_t;
typedef uintptr_t vaddr_t;
typedef uintptr_t paddr_t;

typedef int kobj_id;

typedef uint32_t lk_time_t;
typedef unsigned long long lk_bigtime_t;
#define INFINITE_TIME UINT32_MAX

#define TIME_GTE(a, b) ((int32_t)((a) - (b)) >= 0)
#define TIME_LTE(a, b) ((int32_t)((a) - (b)) <= 0)
#define TIME_GT(a, b) ((int32_t)((a) - (b)) > 0)
#define TIME_LT(a, b) ((int32_t)((a) - (b)) < 0)

enum handler_return {
    INT_NO_RESCHEDULE = 0,
    INT_RESCHEDULE,
};

#ifdef __LP64__
typedef signed long int ssize_t;
#else
typedef signed int ssize_t;
#endif

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

