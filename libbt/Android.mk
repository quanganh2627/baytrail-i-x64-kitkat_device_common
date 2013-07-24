 ######################################################################
 # Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 #
 # This software is licensed under the terms of the GNU General Public
 # License version 2, as published by the Free Software Foundation, and
 # may be copied, distributed, and modified under those terms.
 #
 # This program is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 #####################################################################

LOCAL_PATH := $(call my-dir)

#userial_vendor_test

include $(CLEAR_VARS)

BDROID_DIR := $(TOP_DIR)external/bluetooth/bluedroid

LOCAL_SRC_FILES := src/test_hs.c

LOCAL_SHARED_LIBRARIES := \
        libcutils

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(bdroid_C_INCLUDES)

LOCAL_MODULE := bt_vendor_test
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := intel

include $(BUILD_EXECUTABLE)

# libbt-vendor.so

include $(CLEAR_VARS)

BDROID_DIR := $(TOP_DIR)external/bluetooth/bluedroid

LOCAL_SRC_FILES := \
        src/bt_vendor.c \
        src/hardware.c \
        src/userial_vendor.c \
        src/upio.c \
        src/conf.c

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(BDROID_DIR)/hci/include

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog

LOCAL_MODULE := libbt-vendor
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_OWNER := intel
#LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)

include $(LOCAL_PATH)/vnd_buildcfg.mk

include $(BUILD_SHARED_LIBRARY)

include $(LOCAL_PATH)/conf/intel/sofia_3g/Android.mk
