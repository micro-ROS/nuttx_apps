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
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <drive_base_msgs/msg/trv_command.h>
#include <rcutils/time.h> 
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
    const drive_base_msgs__msg__TRVCommand * cmd = (const drive_base_msgs__msg__TRVCommand *)msgin;
    if(cmd == nullptr)
        return;

    numberMsgCmdVel++;

    // check message age
    rcutils_time_point_value_t msgtime = RCUTILS_S_TO_NS(cmd->header.stamp.sec) + cmd->header.stamp.nanosec;
    rcutils_time_point_value_t curtime;
    rcutils_system_time_now(&curtime);
    rcutils_duration_value_t duration_ns = curtime - msgtime;
    long duration_ms = RCUTILS_NS_TO_MS(duration_ns);
    //fprintf(stdout, "Cur time ns %llu, msg time ns %llu, difference %lld / %ld\n",
    //    curtime, msgtime, duration_ns, duration_ms);
    
    const unsigned int period = cmd->header.expected_period != 0 ? cmd->header.expected_period : 100;
    if(labs(duration_ms) > period/2) {
        fprintf(stderr, "WARN: Ignoring outdated message (age %ld ms > %d ms [period/2])\n", duration_ms, period/2);
        return;
    }
 
    ROS_DEBUG("Received speed cmd %f/%f\n", cmd->translational_velocity, cmd->rotational_velocity);
    r->setSpeed(cmd->translational_velocity, cmd->rotational_velocity);
    r->sendControls();
}

void* kobuki_run(void *np) {
    KobukiRobot robot;
    r = &robot;
    KobukiNode *node = (KobukiNode*)np;
    try {
        robot.connect("/dev/ttyS1");

        struct pollfd pf = robot.getPollFD();
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
    } catch(const std::exception& ex) {
        fprintf(stderr, "Error on reading from robot %s\n", ex.what());
    }
}

extern "C"
{
#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int kobuki_main(int argc, char* argv[]) // name must match '$APPNAME_main' in Makefile 
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

        //create subscription
        const char * cmd_vel_topic_name = "drive_cmd";
        rcl_subscription_t sub_cmd_vel = rcl_get_zero_initialized_subscription();
        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
        
        subscription_ops.qos =  {
                    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                    1,
                    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                    false
        };
        const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(drive_base_msgs, msg, TRVCommand);
        
        CHECK_RET(rcl_subscription_init(
            &sub_cmd_vel,
            &(node.node),
            sub_type_support,
            cmd_vel_topic_name,
            &subscription_ops))


        // initialize and configure executor
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        rcl_allocator_t allocator = rcl_get_default_allocator();
        CHECK_RET(rclc_executor_init(
            &executor,
            &node.context,
            1,
            &allocator))

        // add subscription for topic 'cmd_vel'
        drive_base_msgs__msg__TRVCommand msg;
        CHECK_RET(rclc_executor_add_subscription(
            &executor,
            &sub_cmd_vel,
            &msg,
            commandVelCallback,
            ON_NEW_DATA))

        while (true) {
            node.publish_status_info();
            CHECK_RET(rclc_executor_spin_some(&executor,RCL_MS_TO_NS(timeout_ms)))
        }
	} catch(const std::exception& ex) {
        printf("%s\n", ex.what());
	} catch(...) {
        printf("Caught unknown error");
	}
    
    return 0;
    
  } // main

} // extern "C"

