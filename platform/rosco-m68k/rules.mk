LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

ARCH := m68k
M68K_CPU := 68010
WITH_MMU ?= 0
LK_HEAP_IMPLEMENTATION ?= dlmalloc
WITH_LINKER_GC ?= true

MODULE_DEPS += lib/cbuf

#MODULE_SRCS += $(LOCAL_DIR)/goldfish_rtc.c
#MODULE_SRCS += $(LOCAL_DIR)/goldfish_tty.c
#MODULE_SRCS += $(LOCAL_DIR)/pic.c
MODULE_SRCS += $(LOCAL_DIR)/duart.c
MODULE_SRCS += $(LOCAL_DIR)/platform.c

MEMBASE ?= 0x00002000 # 8k. Just off the end of firmware reserved areas
MEMSIZE ?= 0x00100000 # 1MB

# relocate ourself from the load address (0x40000)
GLOBAL_DEFINES += ARCH_DO_RELOCATION=1

# we can revert to a poll based uart spin routine
GLOBAL_DEFINES += PLATFORM_SUPPORTS_PANIC_SHELL=1

# we will find the memory size by probing it
GLOBAL_DEFINES += NOVM_DEFAULT_ARENA=0

# we will find the memory size by probing it
GLOBAL_DEFINES += TARGET_HAS_DEBUG_LED=1

include make/module.mk
