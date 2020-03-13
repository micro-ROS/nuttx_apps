// Copyright (c) 2018, 2019 Robert Bosch GmbH
// Author: Ingo Lütkebohle <ingo.luetkebohle@de.bosch.com>
// Author: Jan Staschulat <jan.staschulat@de.bosch.com>
// Copyright (c) 2015, Lab-RoCoCo
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

#include <nuttx/config.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/init.h>
#include "platform/cxxinitialize.h"


//#include <ros/ros.h>
//#include <ros/callback_queue.h>

// ROS message types
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>

// include message dependencies
// for time ??? #include "builtin_interfaces/msg/time__struct.h"

// strings in imu message : frame_id
#include "rosidl_generator_c/string.h"

#include "kobuki_robot.h"

#define _USE_MATH_DEFINES
#include <cmath>
//#include <iostream>
//#include <tf/transform_datatypes.h>

#include "uros/ros_util.h"
#include "kobuki_node.h"
#include <poll.h>

using namespace kobuki;

using namespace std;

int numberMsgCmdVel = 0;

struct mallinfo mi;
static void
display_mallinfo(void)
{

#ifdef CONFIG_CAN_PASS_STRUCTS
  mi = mallinfo();
#else
  mallinfo(&mi);
#endif

  printf("Total non-mmapped bytes (arena):       %d\n", mi.arena);
  printf("# of free chunks (ordblks):            %d\n", mi.ordblks);
  printf("Size of largest free chunk (mxordblk)  %d\n", mi.mxordblk);
  printf("Total allocated space (uordblks):      %d\n", mi.uordblks);
  printf("Total free space (fordblks):           %d\n", mi.fordblks);
}

KobukiRobot *r;

void commandVelCallback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
    numberMsgCmdVel++;

    if ( twist != NULL ) {
        ROS_DEBUG("Received speed cmd %f/%f\n", (float)twist->linear.x, (float)twist->angular.z);
        r->setSpeed((float)twist->linear.x, (float)twist->angular.z);
        r->sendControls();
    } else {
        ROS_ERROR("Error in callback commandVelCallback Twist message expected but got %p!\n", msgin);
    }
}

void* kobuki_run(void *np) {
    KobukiRobot robot;
    r = &robot;
    KobukiNode *node = (KobukiNode*)np;
    robot.connect("/dev/ttyS0");

    struct pollfd pf = { .fd = robot._serial_fd, .events = POLLIN, .revents = 0 };
    int32_t packetCount = 0, count = 0;
    while(true) {
        struct timespec ts;        
        poll(&pf, 1, 0);
        robot.receiveData(ts);        
        if(packetCount != robot.packetCount()) {
            packetCount = robot.packetCount();
            ++count;
            if((count % 5) == 0) {
                node->update_state(ts, robot);
            }
        }                
    }
}

constexpr double kobuki_max_angular_velocity = 1.00;
constexpr double kobuki_max_linear_velocity = 0.35;
constexpr double kobuki_damping_constant = 0.995;

extern "C"
{
#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int kobuki_tof_main(int argc, char* argv[]) // name must match '$APPNAME_main' in Makefile 
                                        // and CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME
#endif
  {
    pthread_t kobuki_thread;
    
    try {
        rcl_ret_t          rc;
        int result = 0;
        const uint32_t timeout_ms = 10;

        printf("Turtlebot2 ('Kobuki') driver\n");

        KobukiNode node(argc, argv, "kobuki_node");
        
        result = pthread_create(&kobuki_thread, NULL, &kobuki_run, &node);
        if(result != 0) {
            fprintf(stderr, "Failed to create kobuki thread: %d.\n", result);
        }

        //create publisher
        const char* pose_topic = "robot_pose";

        rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
        const rosidl_message_type_support_t * pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
        rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

        CHECK_RET(rcl_publisher_init(
            &pub_odom,
            &(node.node),
            pub_type_support,
            pose_topic,
            &pub_opt))

        //create subscription
        const char * cmd_vel_topic_name = "cmd_vel";
        rcl_subscription_t sub_cmd_vel = rcl_get_zero_initialized_subscription();
        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
        
        subscription_ops.qos =  {
                    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                    1,
                    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                    false
        };
        const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
        
        CHECK_RET(rcl_subscription_init(
            &sub_cmd_vel,
            &(node.node),
            sub_type_support,
            cmd_vel_topic_name,
            &subscription_ops))

        //create subscription
        const char * tof_trigger_topic_name = "/sensors/tof/trigger";
        rcl_subscription_t sub_tof_trigger = rcl_get_zero_initialized_subscription();
        subscription_ops = rcl_subscription_get_default_options();
        const rosidl_message_type_support_t * sub_type_support_trigger = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
        
        CHECK_RET(rcl_subscription_init(
            &sub_tof_trigger,
            &(node.node),
            sub_type_support_trigger,
            tof_trigger_topic_name,
            &subscription_ops));

        // get empty wait set
        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        CHECK_RET(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &(node.context), rcl_get_default_allocator()))
        
        bool invisible_wall = false;

        geometry_msgs__msg__Twist msg = {0};
        while(true) {
            // set rmw fields to NULL
            CHECK_RET(rcl_wait_set_clear(&wait_set));

            size_t index_vel;
            CHECK_RET(rcl_wait_set_add_subscription(&wait_set, &sub_cmd_vel, &index_vel));

            size_t index_tof;
            CHECK_RET(rcl_wait_set_add_subscription(&wait_set, &sub_tof_trigger, &index_tof));

            rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
            
            if (wait_set.subscriptions[index_tof]){
                std_msgs__msg__Bool msg_tof;
                rc = rcl_take(&sub_tof_trigger, &msg_tof, NULL, NULL);
                invisible_wall = msg_tof.data;
            }

            if (wait_set.subscriptions[index_vel] ){
                rmw_message_info_t        messageInfo;
                rc = rcl_take(&sub_cmd_vel, &msg, &messageInfo, NULL);
                if(rc != RCL_RET_OK) {
                    if(rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
                        fprintf(stderr, "error return on rcl_take: %d\n", rc);
                        PRINT_RCL_ERROR(rcl_take);
                    }
                    continue;
                }
                
                msg.angular.z =
                    std::min(
                        kobuki_max_angular_velocity,
                        std::max(-kobuki_max_angular_velocity, msg.angular.z));
                msg.linear.x =
                    std::min(
                        kobuki_max_linear_velocity,
                        std::max(-kobuki_max_linear_velocity, msg.linear.x));
            }
            else
            {
                msg.angular.z *= kobuki_damping_constant;
                msg.linear.x *= kobuki_damping_constant;
            }

            if (invisible_wall || r->_cliff || r->_bumper){
                msg.angular.z = 0;
                msg.linear.x = (msg.linear.x > 0) ? 0 : msg.linear.x;
            }

            commandVelCallback( &msg );
        }
        WARN_RET(rcl_wait_set_fini(&wait_set));
	} catch(const std::exception& ex) {
        printf("%s\n", ex.what());
	} catch(...) {
        printf("Caught unknown error");
	}
    
    return 0;
    
  } // main

} // extern "C"
