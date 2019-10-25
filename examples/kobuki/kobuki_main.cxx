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

#include "ros_util.h"
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

void commandVelCallback(const void * msgin){ //TwistConstPtr
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  numberMsgCmdVel++;
  //printf("cmd_vel received(#%d)\n", numberMsgCmdVel);

  if ( twist != NULL ) {
    r->setSpeed((float)twist->linear.x, (float)twist->angular.z);
    r->sendControls();
  } else {
        printf("Error in callback commandVelCallback Twist message expected!\n");
  }
}

#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
    fprintf(stderr, "[" #rclc "] error in " #rcl ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

void* kobuki_run(void *np)
{
    KobukiRobot robot;
    r = &robot;
    KobukiNode *node = (KobukiNode*)np;
    robot.connect("/dev/ttyS1");

    struct pollfd pf = { .fd = robot._serial_fd, .events = POLLIN, .revents = 0 };
    uint32_t packetCount = 0, count = 0;
    while( true ){ // ros::ok() did not work on Olimex with micro-ROS
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

        printf("Turtlebot2 embedded kobuki driver\n");

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

        rc = rcl_publisher_init(
            &pub_odom,
            &(node.node),
            pub_type_support,
            pose_topic,
            &pub_opt);

        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(create_publisher, rcl_publisher_init);
            printf("Failed to create publisher: %s.\n", pose_topic);
            return -1;
        } else {
            printf("Created publisher: %s\n", pose_topic);
        }

        //create subscription
        const char * cmd_vel_topic_name = "cmd_vel";
        rcl_subscription_t sub_cmd_vel = rcl_get_zero_initialized_subscription();
        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
        
        subscription_ops.qos = {
                    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                    10,
                    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                    false
            };
        const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
        
        rc = rcl_subscription_init(
            &sub_cmd_vel,
            &(node.node),
            sub_type_support,
            cmd_vel_topic_name,
            &subscription_ops);

        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(create_subscription, rcl_subscription_init);
            printf("Failed to create subscriber: cmd_vel.\n");
            return -1;
        } else {
            printf("Created subscriber %s:\n",cmd_vel_topic_name);
        }

        // get empty wait set
        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_init);
            return -1;
        }
        while(true) {
            // set rmw fields to NULL
            rc = rcl_wait_set_clear(&wait_set);
            if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_clear_subscriptions);
                break;
            }

            size_t index = 0; // is never used - denotes the index of the subscription in the storage container
            rc = rcl_wait_set_add_subscription(&wait_set, &sub_cmd_vel, &index);
            if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_add_subscription);
                break;
            }

            rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
            if (rc == RCL_RET_TIMEOUT) {
                continue;
            }

            if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
                PRINT_RCL_ERROR(spin_node_once, rcl_wait);
                continue;
            }
            
            if (wait_set.subscriptions[0] ){
                geometry_msgs__msg__Twist msg;
                rmw_message_info_t        messageInfo;
                rc = rcl_take(&sub_cmd_vel, &msg, &messageInfo);
                if(rc != RCL_RET_OK) {
                    continue;
                }

                commandVelCallback( &msg );
            } else {
                //sanity check
                fprintf(stderr, "[spin_node_once] wait_set returned empty.\n");
            }

            node.publish_status_info();
        }        
        rcl_wait_set_fini(&wait_set);
	} catch(const std::exception& ex) {
        printf("%s\n", ex.what());
	} catch(...) {
        printf("Caught unknown error");
	}
    
    return 0;
    
  } // main

} // extern "C"

