//
// Created by juan on 31/01/17.
//

#include "move_base_jni.h"
#include "move_base_util.h"
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_ekumen_tangobot_nodes_MoveBaseNode_execute
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

    move_base_util::MoveBaseNodeExecutor moveBaseNodeExecutor;
    moveBaseNodeExecutor.Execute(master.c_str(), host.c_str(), node.c_str());

    return;
}

JNIEXPORT void JNICALL Java_com_ekumen_tangobot_nodes_MoveBaseNode_shutdown
        (JNIEnv *, jobject) {

}

#ifdef __cplusplus
}
#endif