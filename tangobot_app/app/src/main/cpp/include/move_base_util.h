//
// Created by juan on 01/02/17.
//

#ifndef TANGOBOT_APP_MOVE_BASE_UTIL_H
#define TANGOBOT_APP_MOVE_BASE_UTIL_H

enum {
    ROS_INIT_SUCCESS,
    ROS_INIT_ERROR
};

namespace move_base_util {

class MoveBaseNodeExecutor {
    public:
        MoveBaseNodeExecutor();
        ~MoveBaseNodeExecutor();

        void Execute(const char* master_uri, const char* host_ip, const char* node_name);
        void Shutdown();
};

} // namespace move_base_util

#endif //TANGOBOT_APP_MOVE_BASE_UTIL_H
