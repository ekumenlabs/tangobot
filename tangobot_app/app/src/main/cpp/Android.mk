LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := move_base_jni
LOCAL_SRC_FILES := src/move_base_jni.cpp src/move_base_util.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS  += --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS := -landroid -llog
LOCAL_WHOLE_STATIC_LIBRARIES := liblayers libdwa_local_planner libclear_costmap_recovery librotate_recovery libglobal_planner libnavfn libtrajectory_planner_ros libcarrot_planner libmove_slow_and_clear
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk

include $(BUILD_SHARED_LIBRARY)

# This file should contain the import path for roscpp_android_ndk buildscript.
# For example: #$(call import-add-path, /home/user/ros-android-ndk/roscpp_android/output)
# The local file shouldn't be commited to the repository.
include $(LOCAL_PATH)/local-properties.mk
$(call import-module, roscpp_android_ndk)

