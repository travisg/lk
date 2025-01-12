LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

# ROMBASE, MEMBASE, and MEMSIZE are required for the linker script
ROMBASE ?= 0x08000000
MEMBASE ?= 0x20000000
# default memsize, specific STM32_CHIP may override this
# and target/project may have already overridden
MEMSIZE ?= 131072

ARCH := arm
ARM_CPU := cortex-m4

# TODO: integrate better with platform/stm32f4xx/CMSIS/stm32f4xx.h
ifeq ($(STM32_CHIP),stm32f407)
GLOBAL_COMPILEFLAGS += -DSTM32F40_41xxx=1
FOUND_CHIP := true
endif
ifeq ($(STM32_CHIP),stm32f417)
FOUND_CHIP := true
GLOBAL_COMPILEFLAGS += -DSTM32F40_41xxx=1
endif
ifeq ($(STM32_CHIP),stm32f429)
FOUND_CHIP := true
GLOBAL_COMPILEFLAGS += -DSTM32F429_439xx=1
endif

ifeq ($(FOUND_CHIP),)
$(error unknown STM32F4xx chip $(STM32_CHIP))
endif

GLOBAL_DEFINES += \
    NOVM_MAX_ARENAS=2

GLOBAL_INCLUDES += \
	$(LOCAL_DIR)/include/dev

MODULE_SRCS += \
	$(LOCAL_DIR)/init.c \
	$(LOCAL_DIR)/vectab.c \
	$(LOCAL_DIR)/gpio.c \
	$(LOCAL_DIR)/timer.c \
	$(LOCAL_DIR)/debug.c \
	$(LOCAL_DIR)/uart.c \
	$(LOCAL_DIR)/flash.c \
	$(LOCAL_DIR)/sdram.c

# use a two segment memory layout, where all of the read-only sections 
# of the binary reside in rom, and the read/write are in memory. The 
# ROMBASE, MEMBASE, and MEMSIZE make variables are required to be set 
# for the linker script to be generated properly.
#
LINKER_SCRIPT += \
	$(BUILDDIR)/system-twosegment.ld

MODULE_DEPS += \
	platform/stm32 \
	platform/stm32f4xx/STM32F4xx_StdPeriph_Driver \
	arch/arm/arm-m/systick \
	lib/cbuf \
	lib/bio

include make/module.mk
