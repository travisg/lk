LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += $(LOCAL_DIR)/virtio-bus.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio-device.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio-mmio-bus.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio-pci-bus.cpp
MODULE_SRCS += $(LOCAL_DIR)/virtio.cpp

include make/module.mk
