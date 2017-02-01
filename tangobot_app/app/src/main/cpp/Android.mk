LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := move_base_jni
LOCAL_SRC_FILES := src/move_base_jni.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS  += --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS := -landroid -llog
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk

include $(BUILD_SHARED_LIBRARY)

# This file should contain the import path for roscpp_android_ndk buildscript.
# For example: #$(call import-add-path, /home/user/ros-android-ndk/roscpp_android/output)
# The local file shouldn't be commited to the repository.
include $(LOCAL_PATH)/local-properties.mk
$(call import-module, roscpp_android_ndk)

