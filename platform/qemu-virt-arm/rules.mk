LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

ifeq ($(ARCH),)
ARCH := arm64
endif
ifeq ($(ARCH),arm64)
ARM_CPU ?= cortex-a53
endif
ifeq ($(ARCH),arm)
ARM_CPU ?= cortex-a15
endif
WITH_SMP ?= 1

LK_HEAP_IMPLEMENTATION ?= dlmalloc

MODULE_SRCS += \
    $(LOCAL_DIR)/debug.c \
    $(LOCAL_DIR)/platform.c \
    $(LOCAL_DIR)/secondary_boot.S \

MEMBASE := 0x40000000
MEMSIZE ?= 0x08000000   # 512MB
KERNEL_LOAD_OFFSET := 0x100000 # 1MB

MODULE_DEPS += \
    lib/cbuf \
    lib/fdtwalk \
    dev/interrupt/arm_gic \
    dev/timer/arm_generic \
    dev/virtio/block \
    dev/virtio/gpu \
    dev/virtio/net \
    dev/pl011 \

GLOBAL_DEFINES += \
    MEMBASE=$(MEMBASE) \
    MEMSIZE=$(MEMSIZE) \
    PLATFORM_SUPPORTS_PANIC_SHELL=1 \
    CONSOLE_HAS_INPUT_BUFFER=1

GLOBAL_DEFINES += MMU_WITH_TRAMPOLINE=1 \

LINKER_SCRIPT += \
    $(BUILDDIR)/system-onesegment.ld

include make/module.mk
