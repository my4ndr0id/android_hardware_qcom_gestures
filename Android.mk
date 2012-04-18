ifeq ($(strip $(BOARD_USES_QCOM_HARDWARE)), true)
ifneq ($(BUILD_TINY_ANDROID),true)
ifeq ($(call is-board-platform,msm8960),true)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(strip $(TARGET_USES_ION)),true)
LOCAL_CFLAGS += -DUSE_ION
endif

LOCAL_HAL_FILES := \
        mm_sock.c \
        QualcommGestures.cpp \
        GesturesMgr.cpp \
        GestureComm.cpp \
        GestureSession.cpp \
        GestureNotifier.cpp \
        GestureParameters.cpp

LOCAL_SRC_FILES := $(LOCAL_HAL_FILES)

LOCAL_C_INCLUDES := $(TARGET_OUT_HEADERS)/mm-gestures
LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/media
LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_C_INCLUDES += hardware/qcom/display/libgralloc \
                    hardware/qcom/display/libgenlock \
                    hardware/qcom/media/libstagefrighthw

LOCAL_SHARED_LIBRARIES:= libutils libui libgesture_client liblog libcutils
LOCAL_SHARED_LIBRARIES+= libgenlock libbinder
LOCAL_SHARED_LIBRARIES+= libdl

LOCAL_CFLAGS += -include bionic/libc/kernel/common/linux/socket.h

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE:= gestures.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)

endif # msm8960
endif # BUILD_TINY_ANDROID
endif # BOARD_USES_QCOM_HARDWARE
