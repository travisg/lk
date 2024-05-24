LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/virtio-net.cpp

MODULE_DEPS += \
	dev/virtio \
	lib/minip

include make/module.mk
