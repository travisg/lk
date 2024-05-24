LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/client.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio-9p.cpp
MODULE_SRCS += $(LOCAL_DIR)/protocol.cpp

MODULE_DEPS += \
	dev/virtio

include make/module.mk
