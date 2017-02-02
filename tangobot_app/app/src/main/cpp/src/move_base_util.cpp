//
// Created by juan on 01/02/17.
//

#include "move_base_util.h"
#include <move_base/move_base.h>
#include <android/log.h>
#include <ros/ros.h>
#include <sstream>
#include <ostream>
#include <string>

namespace move_base_util {

/*
 * Auxiliary log functions
 */
static void log(std::string msg, int priority = ANDROID_LOG_INFO) {
    __android_log_print(priority, "move_base_native", msg.c_str(), NULL);
}

static void log(std::ostream& msg, int priority = ANDROID_LOG_INFO) {
    std::string string_message = static_cast<std::ostringstream&>(msg).str();
    log(string_message, priority);
}

int InitRos(const char* master_uri, const char* host_ip,
            const char* node_name) {
    int argc = 3;
    char* master_uri_copy = strdup(master_uri);
    char* host_ip_copy = strdup(host_ip);
    char* argv[] = {"nothing_important" , master_uri_copy, host_ip_copy};

    log(std::ostringstream()
        << "\nMaster: " << master_uri_copy << "\n"
        << "Host: " << host_ip_copy << "\n"
        << "Node: " << node_name);
    ros::init(argc, &argv[0], node_name);
    free(master_uri_copy);
    free(host_ip_copy);
    if (ros::master::check()) {
        log("ROS MASTER IS UP!");
    } else {
        log("NO ROS MASTER!");
        return ROS_INIT_ERROR;
    }
    return ROS_INIT_SUCCESS;
}

MoveBaseNodeExecutor::MoveBaseNodeExecutor() {}

MoveBaseNodeExecutor::~MoveBaseNodeExecutor() {}

void MoveBaseNodeExecutor::Execute(const char* master_uri, const char* host_ip, const char* node_name) {
    int result = InitRos(master_uri, host_ip, "move_base_android"); // Todo on rosjava update: use node_name as parameter. In current NativeNodeMain, libname is used
    if (result == ROS_INIT_ERROR) {
        return;
    }

    ros::NodeHandle n;

    tf::TransformListener tf(ros::Duration(10));
    move_base::MoveBase move_base(tf);

    ros::WallRate loop_rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MoveBaseNodeExecutor::Shutdown() {}

}