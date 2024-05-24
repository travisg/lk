LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/virtio-gpu.cpp

MODULE_DEPS += \
	dev/virtio \
	lib/gfx

include make/module.mk
