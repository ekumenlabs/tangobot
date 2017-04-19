// Copyright 2017 Ekumen, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

        int Execute(const char* master_uri, const char* host_ip, const char* node_name);
        int Shutdown();
};

} // namespace move_base_util

#endif //TANGOBOT_APP_MOVE_BASE_UTIL_H
