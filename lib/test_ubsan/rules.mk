LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/test_ubsan.c \
	$(LOCAL_DIR)/test_ubsan.cpp

MODULE_DEPS += \
	lib/ubsan

MODULE_COMPILEFLAGS += -fsanitize=undefined

include make/module.mk
