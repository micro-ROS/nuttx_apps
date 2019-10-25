// Copyright (c) 2018, 2019 Robert Bosch GmbH
// Author: Ingo Luetkebohle <ingo.luetkebohle@de.bosch.com>
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of thin_drivers nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef __THIN_KOBUKI_NODE_H__
#define __THIN_KOBUKI_NODE_H__

#include <pthread.h>
#include <time.h>
#include <rcl/rcl.h>
#include <drive_base_msgs/msg/base_info.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/float32.h>
#include "kobuki_robot.h"

namespace kobuki {
    // keeping as a struct for easier "C" interfacing
    struct KobukiNode {
        KobukiNode(int argc, char* argv[], const char *node_name);
        ~KobukiNode();

        void update_state(const struct timespec &ts, const KobukiRobot& robot);

        // publish status info, e.g. BatteryState
        void publish_status_info();

        rcl_context_t context;
        rcl_node_t node;

        rcl_publisher_t pub_base_info;
        drive_base_msgs__msg__BaseInfo msg_base_info;
        const rosidl_message_type_support_t *ts_base_info;

        rcl_guard_condition_t kobuki_guard;

        bool dirty;

        pthread_mutex_t update_mutex;
    };
}

#endif  // __THIN_KOBUKI_NODE_H__