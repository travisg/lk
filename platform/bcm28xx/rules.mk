LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

WITH_SMP := 1
#LK_HEAP_IMPLEMENTATION ?= dlmalloc

MODULE_DEPS := \
	dev/timer/arm_generic \
	lib/cbuf


#lib/bio \
	lib/cbuf \
	lib/minip \
	dev/interrupt/arm_gic \
	dev/timer/arm_cortex_a9

MODULE_SRCS += \
	$(LOCAL_DIR)/gpio.c \
	$(LOCAL_DIR)/intc.c \
	$(LOCAL_DIR)/platform.c \
	$(LOCAL_DIR)/mailbox.c \


MEMBASE := 0x00000000

GLOBAL_DEFINES += \
	ARM_ARCH_WAIT_FOR_SECONDARIES=1

LINKER_SCRIPT += \
	$(BUILDDIR)/system-onesegment.ld

ifeq ($(TARGET),rpi2)
ARCH := arm
ARM_CPU := cortex-a7
# put our kernel at 0x80000000
KERNEL_BASE = 0x80000000
KERNEL_LOAD_OFFSET := 0x00008000
MEMSIZE ?= 0x10000000 # 256MB
SMP_CPU_ID_BITS := 8
GLOBAL_DEFINES += \
	BCM2836=1

MODULES += \
	dev/pl011

else ifeq ($(TARGET),rpi3)
ARCH := arm64
ARM_CPU := cortex-a53

KERNEL_LOAD_OFFSET := 0x00080000
MEMSIZE ?= 0x40000000 # 1GB

GLOBAL_DEFINES += \
    MEMBASE=$(MEMBASE) \
    MEMSIZE=$(MEMSIZE) \
    MMU_WITH_TRAMPOLINE=1 \
	BCM2837=1

MODULE_SRCS += \
	$(LOCAL_DIR)/miniuart.c

MODULE_DEPS += \
		app/shell \
	    app/tests \
	    lib/fdt

endif

include make/module.mk
