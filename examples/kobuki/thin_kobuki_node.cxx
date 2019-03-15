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
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#include <nuttx/init.h>
#include "platform/cxxinitialize.h"


//#include <ros/ros.h>
//#include <ros/callback_queue.h>

// ROS message types
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

using namespace std;

// string constants for topics
const rosidl_generator_c__String ODOM_FRAME_ID ={ "odom", 4, 4 };
const rosidl_generator_c__String IMU_FRAME_ID ={ "gyro_link", 9, 9 };
const rosidl_generator_c__String ODOM_HEADER_FRAME_ID ={ "base_link", 9, 9 };

void kobuki_on_message(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    printf("Subscriber: I heard: [%i]\n", msg->data);
}

// Stop motion if we do not receive a command without this timeout
//const ros::Duration COMMAND_TIMEOUT(0.3);


KobukiRobot robot;
volatile double tv = 0, rv = 0;


//ros::Time last_update;
uros_time_t last_update;


void commandVelCallback(const void * msgin){ //TwistConstPtr
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  if ( twist != NULL ) {
    tv = twist->linear.x;
    rv = twist->angular.z;
    //const char* bytesPtr = (const char*) msgin;
    //printf("msgin: ");
    //for(int i=0; i < 48;++i) {
    //  printf("%02x ", bytesPtr[i]);
    //}
    //printf("\n");
    robot.setSpeed(tv, rv);
    robot.sendControls();
    //last_update = ros::Time::now();
    printf("CommandVelCallback twist received (int) tv=%d rv=%d \n",(int) (twist->linear.x), (int)(twist->angular.z));

    //debug output float CONFIG_LIBC_FLOATINGPOINT=y
    printf("CommandVelCallback twist received (float) tv=%f rv=%lf \n", (float) (twist->linear.x), (double) (twist->angular.z));
    } else {
    printf("Error in callback commandVelCallback Twist message expected!\n");
  }
}

/*
void commandVelStampedCallback(const geometry_msgs::TwistStampedConstPtr twist){
  tv = twist->twist.linear.x;
  rv = twist->twist.angular.z;
  robot.setSpeed(tv, rv);
  robot.sendControls();
  //last_update = twist->header.stamp;
  //ros::Time current = ros::Time::now();
  //ROS_INFO_STREAM("Command received " << (current - last_update) << " after sending");
}
*/

/*
void safeStop(const ros::TimerEvent& evt) {
  if((evt.current_real - last_update) > COMMAND_TIMEOUT) {
    robot.setSpeed(0, 0);
    robot.sendControls();
  }
}
*/


extern "C"
{
#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int kobuki_main(int argc, char* argv[]) // name must match '$APPNAME_main' in Makefile (there is also a dependency on CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME)
#endif
  {
    int result = 0;
    rclc_init(argc, argv);

    /*

    //
    // simple publisher and subscriber
    //
    rclc_node_t* node = NULL;
    if (node = rclc_create_node("olimex_node", ""))
    {
        bool pub_ok = false;
	bool sub_ok = false;

	rclc_publisher_t* publisher = NULL;
        if(publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "chatter_Int32", 1))
        {
            printf("Created publisher: chatter_Int32 \n");
	    pub_ok = true;
	}

	rclc_subscription_t* sub = NULL;
        if(sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                                        "blob_Int32", kobuki_on_message, 1, false))
        {
	  printf("Created subscriber : blob_Int32");
	  sub_ok = true;
	}

	if ( (pub_ok == true) && (sub_ok == true) )
	{
            std_msgs__msg__Int32 msg;
            msg.data = 0;
            while (true ) // rclc_ok() 
            {
                printf("Sending chatter_Int32: '%i'\n", msg.data++);
                rclc_publish(publisher, (const void*)&msg);
                rclc_spin_node_once(node, 500);
		if (msg.data == 1000) { msg.data = 0; }
            }

	    // todo clean-up
            rclc_destroy_publisher(publisher);
	    rclc_destroy_subscription(sub);
        }
        else
        {
            printf("Issues creating publisher or subscriber\n");
            result = -1;
        }
        rclc_destroy_node(node);
    }
    else
    {
        printf("Issues creating node\n");
        result = -1;
    }
    return result;

    //
    // simple publisher and subscriber
    //
 */
    
   //(void) argc;
   //(void) argv;
   //rclc_init(1,"");
   printf("Turtlebot2 embedded kobuki driver\n");
   rclc_node_t* node = NULL;
   if (node = rclc_create_node("free_kobuki_node", ""))
    {
	//defines publishers and subscribers
	rclc_publisher_t  * pub_odom                = NULL;
	rclc_publisher_t  * pub_imu                 = NULL;
	rclc_publisher_t  * pub_twist               = NULL;
	rclc_subscription_t * sub_cmd_vel             = NULL;
        //rclc_subscription_t * sub_cmd_vel_stamped     = NULL;

	//defines status variables for creation of publishers and subscribers
	bool pub_odom_ok             = false;
	bool pub_imu_ok              = false;
	bool pub_twist_ok            = false;
	bool sub_cmd_vel_ok          = false;
	//bool sub_cmd_vel_stamped     = false;
        // timer safestop;
      	
	if (pub_odom = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom", 10)){
          pub_odom_ok = true;
          printf("Created publisher: odom.\n");
        } else {
          printf("Failed to create publisher: odom.\n");
        }

        if (pub_imu = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data", 10) )
        {
          pub_imu_ok = true;
          printf("Created publisher: imu_data.\n");
        } else {
          printf("Failed to create publisher: imu_data.\n");
        }


	if (pub_twist = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "kobuki_twist", 10)){
          pub_twist_ok = true;
          printf("Created publisher: kobuki_twist.\n");
        } else {
          printf("Failed to create publisher: kobuki_twist.\n");
        }

	
        if(sub_cmd_vel = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                                        "cmd_vel", commandVelCallback, 1, false))
        {
	  sub_cmd_vel_ok = true;
	  printf("Created subscriber: cmd_vel.\n");
        } else {
	  printf("Failed to create subscriber: cmd_vel.\n");
	}

	/*
        if(sub_cmd_vel = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
                                                        "cmd_vel_stamped", commandVeliStampedCallback, 1, false))
        {
          sub_cmd_vel_stamped_ok = true;
          printf("Created subscriber: cmd_vel_stamped.\n");
        } else {
          print("Failed to create subscriber: cmd_vel_stamped.\n");
        }
	*/
	
	//check correct initialization, otherwise abort
        if ( (pub_odom_ok    == false) || (pub_imu_ok     == false) || 
             (sub_cmd_vel_ok == false)                            )    // || (sub_cmd_vel_ok == false)    )
        {
	  printf("Abort: could not create subscriber or publisher.\n");
	  return -1;
        } else {

	  // connections are properly setup
	  //TODO replace std::string with char var[256]
	  std::string serial_device;
  	  std::string odom_topic;
	  
	  rosidl_generator_c__String odom_frame_id;
	  rosidl_generator_c__String imu_frame_id;
	  
	  std::string command_vel_topic;

	  
          // ros::init(argc, argv, "free_kobuki_node");
  	  // ros::NodeHandle nh("~");
	  // last_update = ros::Time::now();
	  // nh.param("serial_device", serial_device, std::string("/dev/ttyUSB0"));
	  // nh.param("odom_topic", odom_topic, std::string("/odom"));
	  // nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
	  // nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));
	  // nh.param("imu_frame_id", imu_frame_id, std::string("gyro_link"));
 
   	  serial_device     = "/dev/ttyS0";
	  odom_topic        = "odom";
 	  command_vel_topic = "cmd_vel";
          odom_frame_id     = ODOM_FRAME_ID; // "odom";
          imu_frame_id      = IMU_FRAME_ID; // "gyro_link";

 
  	  // cerr << "running with params: ";
	  // cerr << "serial_device: " << serial_device << endl;
	  // cerr << "odom_topic: " << odom_topic << endl;
	  // cerr << "odom_frame_id: " << odom_frame_id << endl;
	  // cerr << "imu_frame_id: " << imu_frame_id << endl;
	  // cerr << "command_vel_topic: " << command_vel_topic << endl;

          robot.connect(serial_device);

	  //ros::Rate r(100);
	  //TODO set rate
	  
  	  nav_msgs__msg__Odometry odom;
	  nav_msgs__msg__Odometry__init(&odom);

  	  sensor_msgs__msg__Imu  imu;
          sensor_msgs__msg__Imu__init(&imu);


	  geometry_msgs__msg__Twist  msg_twist;
          geometry_msgs__msg__Twist__init(&msg_twist);
	  
  	  imu.header.frame_id = imu_frame_id;
  	  odom.header.frame_id = odom_frame_id;

		  
  	  int packet_count = 0;
	  float64 delta    = 0.0;

  	  while( true ){ // ros::ok() did not work on Olimex with micro-ROS
    		//ros::spinOnce();

	        //ros::Time timestamp;
	        uros_time_t timestamp;
	        //robot.receiveData(timestamp);

		//if (packet_count < robot.packetCount()) {
		  // send the odometry
      		  double x,y,theta, vx, vtheta;
      		  //robot.getOdometry(x,y,theta,vx,vtheta);
      		  //odom.header.stamp = timestamp;
      		  odom.header.frame_id = ODOM_HEADER_FRAME_ID; // "base_link";
      		  odom.pose.pose.position.x = x;
      		  odom.pose.pose.position.y = y;
      		  odom.pose.pose.position.z = 0;
      		  double s = sinf ((float) theta/2);//TODO use CONFIG_HAVE_DOUBLE
      		  double c = cosf ((float) theta/2);
      		  odom.pose.pose.orientation.x = 0;
      		  odom.pose.pose.orientation.y = 0;
      		  odom.pose.pose.orientation.z = s;
      		  odom.pose.pose.orientation.w = c;
      		  odom.twist.twist.linear.x = vx;
      		  odom.twist.twist.angular.z = vtheta;
      			
		  //rclc_publish( pub_odom, (const void *) &odom);
		  //printf("Sending odom\n");
	
      		  // imu data
      		  double heading;
      		  //robot.getImu(heading, vtheta);
		  //imu.header.seq = seq; //header.seq does not exist
      		  //imu.header.stamp = timestamp;
      		  //imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading);
      		  //imu.angular_velocity.z = vtheta;
      			
		  //rclc_publish( pub_imu, (const void *) &imu); 
		  //printf("Sending imu\n");

		  // debug - sending a twist message - because odom and imu data produce error messages in nsh shell as well as in ros2-environment
		  msg_twist.linear.x = 1.0 + delta;
		  msg_twist.linear.y = 2.0;
		  msg_twist.linear.z = 3.0;

		  msg_twist.angular.x = 0.1;
		  msg_twist.angular.y = 0.2;
		  msg_twist.angular.z = 0.3 + delta;
		  rclc_publish ( pub_twist, (const void *) & msg_twist);
		  printf("Sending kobuki_twist\n");

		  delta += 0.1;
		  if (delta > 100) { delta = 0.0; }
		  
      		  //packet_count = robot.packetCount();
	       //}

	  //spin once
	  rclc_spin_node_once(node, 1);
          }
        }        
   }
   return 0;
    
  } // main

} // extern "C"

