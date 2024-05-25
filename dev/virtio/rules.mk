LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/virtio.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio-device.cpp

include make/module.mk
