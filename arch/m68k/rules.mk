LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/arch.c
MODULE_SRCS += $(LOCAL_DIR)/asm.S
MODULE_SRCS += $(LOCAL_DIR)/exceptions.c
MODULE_SRCS += $(LOCAL_DIR)/exceptions_asm.S
MODULE_SRCS += $(LOCAL_DIR)/mmu.c
MODULE_SRCS += $(LOCAL_DIR)/start.S
MODULE_SRCS += $(LOCAL_DIR)/thread.c

GLOBAL_DEFINES += SMP_MAX_CPUS=1

# set the default toolchain to microblaze elf and set a #define
ifndef TOOLCHAIN_PREFIX
TOOLCHAIN_PREFIX := m68k-elf-
endif

# select the cpu based on flags the platform/target passes in
M68K_CPU ?= 68040 # default to 040

ifeq ($(M68K_CPU),68000)
ARCH_COMPILEFLAGS := -mcpu=68000
else ifeq ($(M68K_CPU),68010)
ARCH_COMPILEFLAGS := -mcpu=68010
else ifeq ($(M68K_CPU),68020)
ARCH_COMPILEFLAGS := -mcpu=68020
else ifeq ($(M68K_CPU),68030)
ARCH_COMPILEFLAGS := -mcpu=68030
M68K_MMU := 68030
else ifeq ($(M68K_CPU),68040)
ARCH_COMPILEFLAGS := -mcpu=68040
M68K_MMU := 68040
else
$(error add support for selected cpu $(M68K_CPU))
endif

LIBGCC := $(shell $(TOOLCHAIN_PREFIX)gcc $(GLOBAL_COMPILEFLAGS) $(ARCH_COMPILEFLAGS) $(GLOBAL_COMPILEFLAGS) -print-libgcc-file-name)
$(info LIBGCC = $(LIBGCC))

cc-option = $(shell if test -z "`$(1) $(2) -S -o /dev/null -xc /dev/null 2>&1`"; \
	then echo "$(2)"; else echo "$(3)"; fi ;)

# default to no mmu
WITH_MMU ?= 0

ifeq (true, $(call TOBOOL, $(WITH_MMU)))
ifeq ($(M68K_MMU),)
$(error WITH_MMU is set but no M68K_MMU is set)
endif
GLOBAL_DEFINES += M68K_MMU=$(M68K_MMU)
endif

ARCH_OPTFLAGS := -O2

KERNEL_BASE ?= $(MEMBASE)
KERNEL_LOAD_OFFSET ?= 0

GLOBAL_DEFINES += MEMBASE=$(MEMBASE)
GLOBAL_DEFINES += MEMSIZE=$(MEMSIZE)
GLOBAL_DEFINES += M68K_CPU=$(M68K_CPU)
GLOBAL_DEFINES += M68K_CPU_$(M68K_CPU)=1

# potentially generated files that should be cleaned out with clean make rule
GENERATED += \
	$(BUILDDIR)/linker.ld

# rules for generating the linker
$(BUILDDIR)/linker.ld: $(LOCAL_DIR)/linker.ld $(wildcard arch/*.ld) linkerscript.phony
	@echo generating $@
	@$(MKDIR)
	$(NOECHO)sed "s/%MEMBASE%/$(MEMBASE)/;s/%MEMSIZE%/$(MEMSIZE)/;s/%KERNEL_BASE%/$(KERNEL_BASE)/;s/%KERNEL_LOAD_OFFSET%/$(KERNEL_LOAD_OFFSET)/" < $< > $@.tmp
	@$(call TESTANDREPLACEFILE,$@.tmp,$@)

linkerscript.phony:
.PHONY: linkerscript.phony

LINKER_SCRIPT += $(BUILDDIR)/linker.ld

include make/module.mk
