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

#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "netutils/telnetd.h"
#include "netutils/netlib.h"
#include "netutils/dhcpc.h"
#include "nshlib/nshlib.h"

#include "platform/cxxinitialize.h"
#include "rosidl_generator_c/string.h"

#include "kobuki_robot.h"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

static void netinit(void)
{
    struct in_addr addr;
    void *handle;
    uint8_t mac[IFHWADDRLEN];

    mac[0] = 0x00;
    mac[1] = 0xe0;
    mac[2] = 0xde;
    mac[3] = 0xad;
    mac[4] = 0xbe;
    mac[5] = 0xef;
    netlib_setmacaddr("eth0", mac);

    addr.s_addr = 0;
    netlib_set_ipv4addr("eth0", &addr);
    netlib_ifup("eth0");

    addr.s_addr = HTONL(0xc0a80101);
    netlib_set_dripv4addr("eth0", &addr);

    netlib_getmacaddr("eth0", mac);

    handle = dhcpc_open("eth0", &mac, IFHWADDRLEN);
    printf("Getting IP address\n");
    if (handle)
        {
        struct dhcpc_state ds;
        (void)dhcpc_request(handle, &ds);
        netlib_set_ipv4addr("eth0", &ds.ipaddr);

        if (ds.netmask.s_addr != 0)
            {
            netlib_set_ipv4netmask("eth0", &ds.netmask);
            }

        if (ds.default_router.s_addr != 0)
            {
            netlib_set_dripv4addr("eth0", &ds.default_router);
            }

        if (ds.dnsaddr.s_addr != 0)
            {
            netlib_set_ipv4dnsaddr(&ds.dnsaddr);
            }

        dhcpc_close(handle);
        printf("IP: %s\n", inet_ntoa(ds.ipaddr));
        }
}

volatile float tv = 0, rv = 0;
int numberMsgCmdVel = 0;

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

    } 
}

/* START functions from rclc layer (functions.c)*/
#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
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
    netinit();

    KobukiRobot robot;
    r = &robot;

    while(true) {
        int result = 0;

        rcl_context_t      context;         //global static var in rcl
        rcl_init_options_t init_options;    //global static var in rcl
        rcl_ret_t          rc;
        init_options = rcl_get_zero_initialized_init_options();
        
        rc = rcl_init_options_init(&init_options, rcl_get_default_allocator());
        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(init, rcl_init_options_init);
        }

        context = rcl_get_zero_initialized_context();

        rc = rcl_init(0, NULL, &init_options, &context);
        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(init, rcl_init);
        }

        rcl_init_options_fini(&init_options);
    
        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "free_kobuki_node", "", &context, &node_ops);

        if (rc != RCL_RET_OK) {
            PRINT_RCL_ERROR(create_node, rcl_node_init);
        } else {

            //create publisher
            const char* pose_topic = "robot_pose";

            rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
            const rosidl_message_type_support_t * pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
            rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

            rc = rcl_publisher_init(
                &pub_odom,
                &node,
                pub_type_support,
                pose_topic,
                &pub_opt);

            if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(create_publisher, rcl_publisher_init);
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
                &node,
                sub_type_support,
                cmd_vel_topic_name,
                &subscription_ops);

            if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(create_subscription, rcl_subscription_init);
            } else {
                printf("Created subscriber %s:\n",cmd_vel_topic_name);
            }

            std::string serial_device;
            std::string command_vel_topic;

            serial_device     = "/dev/ttyS0";
            command_vel_topic = "cmd_vel";

            robot.connect(serial_device);
            
            geometry_msgs__msg__Vector3 pose;
            geometry_msgs__msg__Vector3__init(&pose);

            int packet_count = 0;
            int count = 0;

            display_mallinfo();

            geometry_msgs__msg__Twist msg;
            geometry_msgs__msg__Twist__init(&msg); 

            msg.linear.x = 0.0;
            msg.angular.z = 0.0;


            while( true ){
                count++;
                uros_time_t timestamp;
                robot.receiveData(timestamp);

                if (packet_count < robot.packetCount()) {
                    packet_count = robot.packetCount();

                    // send the odometry
                    float x,y,theta, vx, vtheta;
                    robot.getOdometry(x,y,theta,vx,vtheta);

                    pose.x = x;
                    pose.y = y;
                    pose.z = theta; // HACK!
                }
                
                if (&pub_odom ){
                    rcl_publish( &pub_odom, (const void *) &pose);
                }	

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

                    if ( wait_set.subscriptions[0] ){
                        rmw_message_info_t        messageInfo;
                        rc = rcl_take(&sub_cmd_vel, &msg, &messageInfo);

                        if (rc != RCL_RET_OK) {
                            PRINT_RCL_ERROR(spin_node_once, rcl_take);
                            _spin_node_exit(&wait_set);
                            break;
                        }
                        
                    }

                    rcl_wait_set_fini(&wait_set);
                
                } while (0);

                if(msg.linear.x > 0 && (robot.bumper(KobukiRobot::Left) || robot.bumper(KobukiRobot::Center) || robot.bumper(KobukiRobot::Right))){
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                }

                commandVelCallback( &msg );
            }        
        }

        rcl_node_fini( &node);
    }

    return 0;
  } // main
} // extern "C"