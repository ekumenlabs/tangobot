//
// Created by juan on 31/01/17.
//

#include "move_base_jni.h"

#include "ros/ros.h"
#include <move_base/move_base.h>
#include <android/log.h>

#ifdef __cplusplus
extern "C" {
#endif

static void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "MOVE_BASE_NDK_EXAMPLE", msg, args);
    va_end(args);
}

JNIEXPORT void JNICALL Java_com_ekumen_tangobot_application_MoveBaseNode_execute
    (JNIEnv* env, jobject /*obj*/, jstring master_uri_value, jstring host_ip_value, jstring node_name_value,
        jobjectArray remapping_objects_value)  {

    const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
    const char* host_ip = env->GetStringUTFChars(host_ip_value, NULL);
    const char* node_name = env->GetStringUTFChars(node_name_value, NULL);

    std::string master = "__master:=" + std::string(master_uri);
    std::string host = "__ip:=" + std::string(host_ip);
    std::string node = std::string(node_name);

    env->ReleaseStringUTFChars(master_uri_value, master_uri);
    env->ReleaseStringUTFChars(host_ip_value, host_ip);
    env->ReleaseStringUTFChars(node_name_value, node_name);

    char* master_copy = strdup(master.c_str());
    char* host_copy = strdup(host.c_str());

    int argc = 4;

    char *argv[] = {"nothing_important" , master_copy,
                    host_copy, "cmd_vel:=navigation_velocity_smoother/raw_cmd_vel"};

    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }

    ros::init(argc, &argv[0], "move_base_native");

    free(master_copy);
    free(host_copy);

    if(ros::master::check()){
        log("ROS MASTER IS UP!");
    } else {
        log("NO ROS MASTER.");
    }
    log(master.c_str());

    ros::NodeHandle n;

    tf::TransformListener tf(ros::Duration(10));
    move_base::MoveBase move_base(tf);

    ros::WallRate loop_rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

JNIEXPORT void JNICALL Java_com_ekumen_tangobot_application_MoveBaseNode_shutdown
        (JNIEnv *, jobject) {

}

#ifdef __cplusplus
}
#endif