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


#include <math.h>

#include "ros_util.h"
#include "kobuki_node.h"
#include <rosidl_generator_c/string_functions.h>
#include <rosidl_generator_c/primitives_sequence_functions.h>

static const float NOMINAL_BATTERY_VOLTAGE = 16.7f;

namespace kobuki
{
    KobukiNode::KobukiNode(int argc, char* argv[], const char* node_name) 
    : context(rcl_get_zero_initialized_context()), node(rcl_get_zero_initialized_node()),
      kobuki_guard(rcl_get_zero_initialized_guard_condition()),
      update_mutex(PTHREAD_MUTEX_INITIALIZER), dirty(false) {
        rcl_init_options_t init_options;    //global static var in rcl
        rcl_ret_t          rc;
        init_options = rcl_get_zero_initialized_init_options();
        rc = rcl_init_options_init(&init_options, rcl_get_default_allocator());
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }

        rc = rcl_init(argc, argv, &init_options, &context);
        rcl_init_options_fini(&init_options);
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }

        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, node_name, "", &context, &node_ops);
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }

        pub_base_info         = rcl_get_zero_initialized_publisher();
        ts_base_info = ROSIDL_GET_MSG_TYPE_SUPPORT(drive_base_msgs, msg, BaseInfo);
        rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

        rc = rcl_publisher_init(
            &pub_base_info,
            &node,
            ts_base_info,
            "base_info",
            &pub_opt);

        if (rc != RCL_RET_OK) {
            rcl_node_fini( &node);
            throw RCLException("Failed to create BaseInfo publisher");
        }

        drive_base_msgs__msg__BaseInfo__init(&msg_base_info);
        msg_base_info.hw_id = 0;
        
        rcl_guard_condition_options_t guard_options = rcl_guard_condition_get_default_options();
        rc = rcl_guard_condition_init(&kobuki_guard, &context, guard_options);
        if(rc != RCL_RET_OK) {
            drive_base_msgs__msg__BaseInfo__fini(&msg_base_info);
            rcl_node_fini( &node);
            throw RCLException("Could not create Kobuki guard");
        }
        
    }

    KobukiNode::~KobukiNode() {
        drive_base_msgs__msg__BaseInfo__fini(&msg_base_info);
        rcl_guard_condition_fini(&kobuki_guard);        
    }

    void KobukiNode::update_state(const struct timespec &ts, const KobukiRobot& robot)
    {
        pthread_mutex_lock(&update_mutex);
        msg_base_info.hw_timestamp = robot._timestamp;
        msg_base_info.stamp.sec = ts.tv_sec;
        msg_base_info.stamp.nanosec = ts.tv_nsec;

        robot.getOdometry(msg_base_info.x, msg_base_info.y, msg_base_info.orientation, 
            msg_base_info.forward_velocity, msg_base_info.rotational_velocity);
        
        msg_base_info.battery_voltage_pct = (uint8_t)roundf((robot.voltage()/NOMINAL_BATTERY_VOLTAGE)*100.0f);
        msg_base_info.power_supply = drive_base_msgs__msg__BaseInfo__POWER_SUPPLY_STATUS_CHARGING ?
            robot.charger() : drive_base_msgs__msg__BaseInfo__POWER_SUPPLY_STATUS_DISCHARGING;
        // diagnostics info
        msg_base_info.overcurrent = robot._overcurrent_flags;
        msg_base_info.blocked = false;  // TODO: read out from laser
        msg_base_info.in_collision = robot._bumper;
        msg_base_info.at_cliff = robot._cliff;

        dirty = true;
        pthread_mutex_unlock(&update_mutex);
    }

    void KobukiNode::publish_status_info()
    {
        pthread_mutex_lock(&update_mutex);
        if(!dirty) {
            pthread_mutex_unlock(&update_mutex);
            return;
        }
        rcl_ret_t rc = rcl_publish(&pub_base_info, &msg_base_info);
        if(rc != RCL_RET_OK) {
            fprintf(stderr, "Error publishing BaseInfo: %s\n", rcutils_get_error_string().str);
        }
        dirty = false;        
        pthread_mutex_unlock(&update_mutex);
    }
}