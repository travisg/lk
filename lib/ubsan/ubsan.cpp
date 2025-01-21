// Copyright 2022-2023 Pedro Falcato
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <lk/bits.h>
#include <lk/compiler.h>
#include <lk/debug.h>

template <u16 size>
struct small_string
{
    char buf[size];

    const char *c_str()
    {
        return buf;
    }
};

extern "C"
{

struct ubsan_source_location
{
    const char *filename;
    u32 line;
    u32 column;
};

struct ubsan_type_descriptor
{
    u16 typekind;
    u16 typeinfo;
    char typename_[];
};

enum ubsan_type_desc_kind
{
    UBSAN_TYPE_DESC_KIND_INTEGER = 0,
    UBSAN_TYPE_DESC_KIND_FLOAT,
    UBSAN_TYPE_DESC_KIND_UNKNOWN = 0xffff
};

#define UBSAN_TYPE_DESC_KIND_SIGNED (1U << 0)

static bool ubsan_type_is_int(const ubsan_type_descriptor *desc)
{
    return desc->typekind == UBSAN_TYPE_DESC_KIND_INTEGER;
}

static bool ubsan_type_is_signed_int(const ubsan_type_descriptor *desc)
{
    return ubsan_type_is_int(desc) && desc->typeinfo & UBSAN_TYPE_DESC_KIND_SIGNED;
}

static bool ubsan_type_is_uint(const ubsan_type_descriptor *desc)
{
    return ubsan_type_is_int(desc) && !(desc->typeinfo & UBSAN_TYPE_DESC_KIND_SIGNED);
}

static size_t ubsan_type_get_int_width(const ubsan_type_descriptor *desc)
{
    assert(ubsan_type_is_int(desc));
    return (1UL << (desc->typeinfo >> 1));
}

enum ubsan_type_kind
{
    UBSAN_TYPE_KIND_LOAD = 0,
    UBSAN_TYPE_KIND_STORE,
    UBSAN_TYPE_KIND_REFERNCE_BINDING,
    UBSAN_TYPE_KIND_MEMBER_ACCESS,
    UBSAN_TYPE_KIND_MEMBER_CALL,
    UBSAN_TYPE_KIND_CONSTRUCTOR_CALL,
    UBSAN_TYPE_KIND_DOWNCAST_POINTER,
    UBSAN_TYPE_KIND_DOWNCAST_REFERENCE,
    UBSAN_TYPE_KIND_UPCAST,
    UBSAN_TYPE_KIND_UPCAST_TO_VIRTUAL_BASE,
    UBSAN_TYPE_KIND_NONNULL_ASSIGN,
    UBSAN_TYPE_KIND_DYNAMIC_OPERATION
};

struct val
{
    size_t val_;
    ubsan_type_descriptor *type_;
    val(size_t v, ubsan_type_descriptor *t) : val_{v}, type_{t}
    {
    }

    small_string<36> to_string() const
    {
        small_string<36> s;
        if (ubsan_type_is_signed_int(type_))
        {
            snprintf(s.buf, 36, "%zd", (ssize_t) val_);
        }
        else if (ubsan_type_is_uint(type_))
        {
            snprintf(s.buf, 36, "%zu", val_);
        }
        else
        {
            snprintf(s.buf, 36, "<unknown>");
        }

        return s;
    }
};

struct ubsan_type_mismatch_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *type;
    size_t alignment;
    u8 type_check_kind;
};

struct ubsan_type_mismatch_data_v1
{
    ubsan_source_location location;
    ubsan_type_descriptor *type;
    u8 log_alignment;
    u8 type_check_kind;
};

static const char *type_check_kinds[] = {"load of",
                                         "store to",
                                         "reference binding to",
                                         "member access within",
                                         "member call on",
                                         "constructor call on",
                                         "downcast of",
                                         "downcast of",
                                         "upcast of",
                                         "cast to virtual base of",
                                         "_Nonnull binding to",
                                         "dynamic operation on"};

void ubsan_report_start(const ubsan_source_location *location, const char *description)
{
    printf("===================================UBSAN error===================================\n");
    printf("ubsan: %s in %s:%d:%d\n", description, location->filename, location->line,
           location->column);
}

static void ubsan_abort()
{
    panic("ubsan!\n");
}

#ifdef CONFIG_UBSAN_ALWAYS_ABORT
#define die_on_every_ubsan (true)
#else
static bool die_on_every_ubsan = false;
#endif

static void ubsan_report_end()
{
    printf("=================================================================================\n");
    if (unlikely(die_on_every_ubsan))
        ubsan_abort();
}

static void do_ubsan_type_mismatch_nullptr(const ubsan_type_mismatch_data *data, size_t ptr)
{
    ubsan_report_start(&data->location, "Null pointer dereference");
    printf("%s null pointer of type %s\n", type_check_kinds[data->type_check_kind],
           data->type->typename_);
    ubsan_report_end();
}

static void do_ubsan_type_mismatch_unaligned(const ubsan_type_mismatch_data *data, size_t ptr)
{
    ubsan_report_start(&data->location, "Unaligned access");
    printf("%s unaligned pointer %p of type %s (alignment %zu)\n",
           type_check_kinds[data->type_check_kind], (void *) ptr, data->type->typename_,
           data->alignment);
    ubsan_report_end();
}

static void do_ubsan_type_mismatch_objsize(const ubsan_type_mismatch_data *data, size_t ptr)
{
    ubsan_report_start(&data->location, "Insufficient object size");
    printf("%s address %p with insufficient space for type %s\n",
           type_check_kinds[data->type_check_kind], (void *) ptr, data->type->typename_);
    ubsan_report_end();
}

__USED void __ubsan_handle_type_mismatch(ubsan_type_mismatch_data *data, size_t ptr)
{
    if (!ptr)
    {
        do_ubsan_type_mismatch_nullptr(data, ptr);
    }
    else if (data->alignment && (ptr & (data->alignment - 1)))
    {
        do_ubsan_type_mismatch_unaligned(data, ptr);
    }
    else
    {
        do_ubsan_type_mismatch_objsize(data, ptr);
    }
}

__USED void __ubsan_handle_type_mismatch_abort(ubsan_type_mismatch_data *data, size_t ptr)
{
    __ubsan_handle_type_mismatch(data, ptr);
    ubsan_abort();
}

__USED void __ubsan_handle_type_mismatch_v1(ubsan_type_mismatch_data_v1 *data, size_t ptr)
{
    ubsan_type_mismatch_data common_data;

    memcpy(&common_data.location, &data->location, sizeof(ubsan_source_location));
    common_data.type = data->type;
    common_data.alignment = 1UL << data->log_alignment;
    common_data.type_check_kind = data->type_check_kind;

    return __ubsan_handle_type_mismatch(&common_data, ptr);
}

__USED void __ubsan_handle_type_mismatch_v1_abort(ubsan_type_mismatch_data_v1 *data, size_t ptr)
{
    __ubsan_handle_type_mismatch_v1(data, ptr);
    ubsan_abort();
}

struct ubsan_ptr_overflow_data
{
    ubsan_source_location location;
};

__USED void __ubsan_handle_pointer_overflow(ubsan_ptr_overflow_data *data, size_t Base, size_t Result)
{
    ubsan_report_start(&data->location, "ptr overflow");
    printf("ptr operation overflowed %p to %p\n", (void *) Base, (void *) Result);
    ubsan_report_end();
}

__USED void __ubsan_handle_pointer_overflow_abort(ubsan_ptr_overflow_data *data, size_t Base,
                                                size_t Result)
{
    __ubsan_handle_pointer_overflow(data, Base, Result);
    ubsan_abort();
}

enum BuiltinCheckKind
{
    BCK_CTZ_ZERO,
    BCK_CLZ_ZERO,
};

struct ubsan_invalid_builtin_data
{
    ubsan_source_location location;
    u8 kind;
};

__USED void __ubsan_handle_invalid_builtin(ubsan_invalid_builtin_data *data)
{
    const char *builtin_func;

    builtin_func = "<unknown>";

    ubsan_report_start(&data->location, "invalid builtin");

    switch (data->kind)
    {
        case BCK_CLZ_ZERO:
            builtin_func = "clz()";
            goto clz_ctz;
        case BCK_CTZ_ZERO:
            builtin_func = "ctz()";
        clz_ctz:
            printf("Passed 0 to %s\n", builtin_func);
            break;
        default:
            printf("Invalid builtin error (Kind %u)\n", data->kind);
            break;
    }

    ubsan_report_end();
}

__USED void __ubsan_handle_invalid_builtin_abort(ubsan_invalid_builtin_data *data)
{
    __ubsan_handle_invalid_builtin(data);
    ubsan_abort();
}

struct ubsan_overflow_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *type;
};

static void ubsan_handle_integer_overflow(const ubsan_overflow_data *data, size_t lhs, size_t rhs,
                                          const char *op)
{
    bool signed_;
    val lhs_v{lhs, data->type};
    val rhs_v{rhs, data->type};

    signed_ = ubsan_type_is_signed_int(data->type);

    ubsan_report_start(&data->location, "Integer overflow");

    printf("%s integer overflow: %s %s %s can't be represented in type %s\n",
           signed_ ? "signed" : "unsigned", lhs_v.to_string().c_str(), op,
           rhs_v.to_string().c_str(), data->type->typename_);
    ubsan_report_end();
}

#define UBSAN_DEFINE_OVERFLOW(name, op)                                                          \
    __USED void __ubsan_handle_##name(ubsan_overflow_data *data, size_t lhs, size_t rhs)         \
    {                                                                                            \
        ubsan_handle_integer_overflow(data, lhs, rhs, op);                                       \
    }                                                                                            \
                                                                                                 \
    __USED void __ubsan_handle_##name##_abort(ubsan_overflow_data *data, size_t lhs, size_t rhs) \
    {                                                                                            \
        ubsan_handle_integer_overflow(data, lhs, rhs, op);                                       \
        ubsan_abort();                                                                           \
    }

UBSAN_DEFINE_OVERFLOW(add_overflow, "+");
UBSAN_DEFINE_OVERFLOW(sub_overflow, "-");
UBSAN_DEFINE_OVERFLOW(mul_overflow, "*");

__USED void __ubsan_handle_negate_overflow(ubsan_overflow_data *data, size_t value)
{
    val v{value, data->type};
    ubsan_report_start(&data->location, "Negation overflow");
    printf("Negation of %s cannot be represented in type %s\n", v.to_string().c_str(),
           data->type->typename_);
    ubsan_report_end();
}

__USED void __ubsan_handle_negate_overflow_abort(ubsan_overflow_data *data, size_t value)
{
    __ubsan_handle_negate_overflow(data, value);
    ubsan_abort();
}

__USED void __ubsan_handle_divrem_overflow(ubsan_overflow_data *data, size_t lhs, size_t rhs)
{
    val lhsv{lhs, data->type};
    val rhsv{lhs, data->type};
    ubsan_report_start(&data->location, "Divrem overflow");

    if (ubsan_type_is_signed_int(data->type) && (ssize_t) rhs == -1)
    {
        printf("division of %s by -1 cannot be represented in type %s\n", lhsv.to_string().c_str(),
               data->type->typename_);
    }
    else
    {
        assert(ubsan_type_is_int(data->type));
        printf("division of %s by zero\n", lhsv.to_string().c_str());
    }

    ubsan_report_end();
}

__USED void __ubsan_handle_divrem_overflow_abort(ubsan_overflow_data *data, size_t lhs, size_t rhs)
{
    __ubsan_handle_divrem_overflow(data, lhs, rhs);
    ubsan_abort();
}

struct ubsan_shift_oob_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *lhs_type;
    ubsan_type_descriptor *rhs_type;
};

void ubsan_handle_shift_out_of_bounds(const ubsan_shift_oob_data *data, ssize_t lhs, ssize_t rhs)
{
    const auto lhs_t = data->lhs_type;
    const auto rhs_t = data->rhs_type;

    ubsan_report_start(&data->location, "shift-oob");

    if (ubsan_type_is_signed_int(rhs_t) && rhs < 0)
    {
        printf("shift exponent %zd is negative\n", rhs);
    }
    else if ((size_t) rhs >= ubsan_type_get_int_width(lhs_t))
    {
        printf("shift exponent %zu is too large for %zu-bit type %s\n", rhs,
               ubsan_type_get_int_width(lhs_t), lhs_t->typename_);
    }
    else
    {
        if (ubsan_type_is_signed_int(lhs_t) && lhs < 0)
        {
            printf("left shift of negative type %s\n", lhs_t->typename_);
        }
        else
        {
            val v{(size_t) lhs, lhs_t};
            printf("left shift of %s by %zu places cannot be represented in type %s\n",
                   v.to_string().c_str(), (size_t) rhs, lhs_t->typename_);
        }
    }

    ubsan_report_end();
}

__USED void __ubsan_handle_shift_out_of_bounds(ubsan_shift_oob_data *data, size_t lhs, size_t rhs)
{
    ubsan_handle_shift_out_of_bounds(data, (ssize_t) lhs, (ssize_t) rhs);
}

__USED void __ubsan_handle_shift_out_of_bounds_abort(ubsan_shift_oob_data *data, size_t lhs,
                                                   size_t rhs)
{
    ubsan_handle_shift_out_of_bounds(data, (ssize_t) lhs, (ssize_t) rhs);
    ubsan_abort();
}

struct ubsan_out_of_bounds_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *array_type;
    ubsan_type_descriptor *index_type;
};

static void ubsan_handle_out_of_bounds(const ubsan_out_of_bounds_data *data, size_t lhs)
{
    val v{lhs, data->index_type};
    ubsan_report_start(&data->location, "out of bounds");
    printf("index %s out of range for type %s (%zx)\n", v.to_string().c_str(),
           data->array_type->typename_, lhs);
    ubsan_report_end();
}

__USED void __ubsan_handle_out_of_bounds(ubsan_out_of_bounds_data *data, size_t lhs)
{
    ubsan_handle_out_of_bounds(data, (ssize_t) lhs);
}

__USED void __ubsan_handle_out_of_bounds_abort(ubsan_out_of_bounds_data *data, size_t lhs)
{
    ubsan_handle_out_of_bounds(data, (ssize_t) lhs);
    ubsan_abort();
}

struct ubsan_unreachable_data
{
    ubsan_source_location location;
};

__USED void __ubsan_handle_builtin_unreachable(ubsan_unreachable_data *data)
{
    ubsan_report_start(&data->location, "execution reached an unreachable program point");
    ubsan_report_end();
    ubsan_abort(); // Not recoverable
}

struct ubsan_nonnull_arg_data
{
    ubsan_source_location location;
    ubsan_source_location attr_location;
    int arg_index;
};

__USED void __ubsan_handle_nonnull_arg(struct ubsan_nonnull_arg_data *data)
{
    ubsan_report_start(&data->location, "nonnull-argument");
    printf("null pointer passed as argument %d, specified non-null\n", data->arg_index);
    ubsan_report_end();
}

__USED void __ubsan_handle_nonnull_arg_abort(struct ubsan_nonnull_arg_data *data)
{
    __ubsan_handle_nonnull_arg(data);
    ubsan_abort();
}

struct ubsan_nonnull_return_data
{
    ubsan_source_location location;
    ubsan_source_location attr_location;
};

__USED void __ubsan_handle_nonnull_return(ubsan_nonnull_return_data *data)
{
    ubsan_report_start(&data->location, "nonnull return");
    printf("Returning a null pointer from function declared to never return null\n");
    ubsan_report_end();
}

__USED void __ubsan_handle_nonnull_return_abort(ubsan_nonnull_return_data *data)
{
    __ubsan_handle_nonnull_return(data);
    ubsan_abort();
}

__USED void __ubsan_handle_nonnull_return_v1(ubsan_nonnull_return_data *data)
{
    __ubsan_handle_nonnull_return(data);
}

__USED void __ubsan_handle_nonnull_return_v1_abort(ubsan_nonnull_return_data *data)
{
    __ubsan_handle_nonnull_return_abort(data);
}

struct ubsan_invalid_value_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *type;
};

__USED void __ubsan_handle_load_invalid_value(ubsan_invalid_value_data *data, size_t value)
{
    val v{value, data->type};
    ubsan_report_start(&data->location, "load invalid value");
    printf("load of value %s, which is not a valid value for type %s\n", v.to_string().c_str(),
           data->type->typename_);
    ubsan_report_end();
}

__USED void __ubsan_handle_load_invalid_value_abort(ubsan_invalid_value_data *data, size_t value)
{
    __ubsan_handle_load_invalid_value(data, value);
    ubsan_abort();
}

struct ubsan_vla_bound_data
{
    ubsan_source_location location;
    ubsan_type_descriptor *type;
};

__USED void __ubsan_handle_vla_bound_not_positive(ubsan_vla_bound_data *data, ssize_t val)
{
    ubsan_report_start(&data->location, "vla bound not positive");
    printf("vla bound not positive (%zd)\n", val);
    ubsan_report_end();
}

__USED void __ubsan_handle_vla_bound_not_positive_abort(ubsan_vla_bound_data *data, ssize_t val)
{
    __ubsan_handle_vla_bound_not_positive(data, val);
    ubsan_abort();
}

struct ubsan_function_type_mismatch_data {
    ubsan_source_location location;
    ubsan_type_descriptor *type;
    val function_ptr;
};

__USED void __ubsan_handle_function_type_mismatch(ubsan_function_type_mismatch_data *data, size_t ptr)
{
    ubsan_report_start(&data->location, "function type mismatch");
    printf("Call function(%p) through pointer with incompatible type %s\n",
           (void *) ptr, data->type->typename_);
    ubsan_report_end();
}

__USED void __ubsan_handle_function_type_mismatch_abort(ubsan_function_type_mismatch_data *data, size_t ptr)
{
    __ubsan_handle_function_type_mismatch(data, ptr);
    ubsan_abort();
}

struct ubsan_alignment_assumption_data {
    struct ubsan_source_location location;
    struct ubsan_source_location assumption_location;
    struct ubsan_type_descriptor *type;
};

__USED void __ubsan_handle_alignment_assumption(ubsan_alignment_assumption_data *data, ssize_t ptr,
                                                ssize_t align, ssize_t offset)
{
    ssize_t real_ptr;
    ubsan_report_start(&data->location, "Alignment Error");
    if (offset)
        printf("assumption of %zd byte alignment (with offset of %zd byte) for pointer of type %s failed",
               align, offset, data->type->typename_);
    else
        printf("assumption of %zd byte alignment for pointer of type %s failed",
               align, data->type->typename_);

    real_ptr = ptr - offset;
    printf(" %saddress is %lu aligned, misalignment offset is %zd bytes\n",
            offset ? "offset " : "", real_ptr ? (1 << ffs(real_ptr)) : 1,
            real_ptr & (align - 1));

    ubsan_report_end();
}

__USED void __ubsan_handle_alignment_assumption_abort(ubsan_alignment_assumption_data *data,
                                                      ssize_t ptr, ssize_t align, ssize_t offset)
{
    __ubsan_handle_alignment_assumption(data, ptr, align, offset);
    ubsan_abort();
}

struct ubsan_float_cast_data {
    struct ubsan_source_location location;
    struct ubsan_type_descriptor *from_type;
    struct ubsan_type_descriptor *to_type;
};

__USED void __ubsan_handle_float_cast_overflow(ubsan_float_cast_data *data, size_t ptr)
{
    ubsan_report_start(&data->location, "Float Cast Overflow");
    printf("value %s is outside of the range of representable value of type %s at %p\n",
            data->from_type->typename_, data->to_type->typename_, (void *) ptr);
    ubsan_report_end();
}

__USED void __ubsan_handle_float_cast_overflow_abort(ubsan_float_cast_data *data, size_t ptr)
{
    __ubsan_handle_float_cast_overflow(data, ptr);
    ubsan_abort();
}

// Note: we used to have __ubsan_handle_function_type_mismatch but it seems to have been dropped
// from both GCC and clang

// Note 2: Do we want full CFI support? UBSAN seems to have dropped the OG bad_cfi_call check

// Note 3: Keep adding needed/wanted stuff
}
