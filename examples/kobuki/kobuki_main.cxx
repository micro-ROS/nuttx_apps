// Copyright (c) 2018, Robert Bosch GmbH
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
#include "thin_kobuki_node.h"

using namespace std;

void kobuki_on_message(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    printf("Subscriber: I heard: [%i]\n", msg->data);
}

// Stop motion if we do not receive a command without this timeout
//const ros::Duration COMMAND_TIMEOUT(0.3);


volatile float tv = 0, rv = 0;
int numberMsgCmdVel = 0;

//ros::Time last_update;
uros_time_t last_update;

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
    tv = (float)twist->linear.x;
    rv = (float)twist->angular.z;
    
    r->setSpeed(tv, rv);
    r->sendControls();

    //printf("CommandVelCallback twist received (int) tv=%d rv=%d \n",(int) (twist->linear.x), (int)(twist->angular.z));

    //debug output float CONFIG_LIBC_FLOATINGPOINT=y
    //printf("cmd_vel (#%d): tv=%f rv=%lf \n", numberMsgCmdVel, (double) (twist->linear.x), (double) (twist->angular.z));
    } else {
    printf("Error in callback commandVelCallback Twist message expected!\n");
  }
  display_mallinfo();
}

/* START functions from rclc layer (functions.c)*/
#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
    fprintf(stderr, "[" #rclc "] error in " #rcl ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

static
inline
void
_spin_node_exit(rcl_wait_set_t * wait_set)
{
    rcl_ret_t rc = rcl_wait_set_fini(wait_set);
    if (rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(spin_node, rcl_wait_set_fini);
    }
}
/* END defines and functions from rclc layer (functions.c) */

using namespace kobuki;

extern "C"
{
#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int kobuki_main(int argc, char* argv[]) // name must match '$APPNAME_main' in Makefile 
                                        // and CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME
#endif
  {
    display_mallinfo();
    try {
        rcl_ret_t          rc;
        KobukiRobot robot;
        r = &robot;
        int result = 0;
        printf("Turtlebot2 embedded kobuki driver\n");

        KobukiNode node(argc, argv);

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

        // connections are properly setup
        //TODO replace std::string with char var[256]
        std::string serial_device;
        std::string command_vel_topic;

        serial_device     = "/dev/ttyS1";
        command_vel_topic = "cmd_vel";

        robot.connect(serial_device);

        //ros::Rate r(100);
        //TODO set rate
        
        geometry_msgs__msg__Vector3 pose;
        geometry_msgs__msg__Vector3__init(&pose);

        int packet_count = 0;
        float64 delta    = 0.0;
        int count = 0;

        display_mallinfo();

        while( true ){ // ros::ok() did not work on Olimex with micro-ROS
            count++;
            uros_time_t timestamp;
            robot.receiveData(timestamp);

            if (packet_count < robot.packetCount()) {
                packet_count = robot.packetCount();

                // send the odometry
                float x,y,theta, vx, vtheta;
                robot.getOdometry(x,y,theta,vx,vtheta);
                /*if((count % 10) == 0) {
                        printf("x=%f, y=%f, theta=%f, vx=%f, vtheta=%f\n",(double) x, (double)y, (double)theta, (double)vx, vtheta);
                }*/
                pose.x = x;
                pose.y = y;
                pose.z = theta; // HACK!

                if((count % 5) == 0) {

                        if (! &pub_odom ){
                            fprintf(stderr, "[rcl_publish] null pointer to publisher");
                            return RCL_RET_INVALID_ARGUMENT;
                        }	
                        rcl_publish( &pub_odom, (const void *) &pose);
                }

                // imu data
                //float heading;
                //robot.getImu(heading, vtheta);
                //imu.header.seq = seq; //header.seq does not exist
                //imu.header.stamp = timestamp;
                //imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading);
                //imu.angular_velocity.z = vtheta;

                //rclc_publish( pub_imu, (const void *) &imu); 
                //printf("Sending imu\n");
                // debug - sending a twist message - because odom and imu data produce error messages in nsh shell as well as in ros2-environment
                //rclc_publish ( pub_twist, (const void *) & msg_twist);
                //printf("Sending kobuki_twist\n");
                }

            //spin once
            //rclc_spin_node_once(node, 10);
            // use do{}while(0) to have port error handling 
            // by replacing 'return' in error detection parts rclc_spin_node_once  by a 'break')
            
            do {
                uint32_t timeout_ms = 10;

                // get empty wait set
                rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
                rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_init);
                    break;
                }

                // set rmw fields to NULL
                rc = rcl_wait_set_clear(&wait_set);
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_clear_subscriptions);
                    _spin_node_exit(&wait_set);
                    break;
                }

                size_t index = 0; // is never used - denotes the index of the subscription in the storage container
                rc = rcl_wait_set_add_subscription(&wait_set, &sub_cmd_vel, &index);
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_add_subscription);
                    _spin_node_exit(&wait_set);
                    break;
                }

                rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
                if (rc == RCL_RET_TIMEOUT) {
                    _spin_node_exit(&wait_set);
                    break;
                }					

                if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(spin_node_once, rcl_wait);
                _spin_node_exit(&wait_set);
                break;
            }

            // if RET_OK => one subscription is received because only ONE subscription has been added
            // just double check here.
            if ( wait_set.subscriptions[0] ){
                geometry_msgs__msg__Twist msg;
                rmw_message_info_t        messageInfo;
                rc = rcl_take(&sub_cmd_vel, &msg, &messageInfo);

                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_take);
                    _spin_node_exit(&wait_set);
                    break;
                }

                // call message callback
                commandVelCallback( &msg );
                
            } else {
                //sanity check
                fprintf(stderr, "[spin_node_once] no subscription received.\n");
            }
            rcl_wait_set_fini(&wait_set);
            
            } while (0);
        }        
	} catch(const std::exception& ex) {
        printf("%s\n", ex.what());
	} catch(...) {
        printf("Caught unknown error");
	}

    return 0;
    
  } // main

} // extern "C"

