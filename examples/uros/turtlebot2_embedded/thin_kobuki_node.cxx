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
#include <stdio.h>

// copied from helloxx_main.cxx example for C++ programs
#include <cstdio>
#include <debug.h>
#include <nuttx/init.h>
#include "platform/cxxinitialize.h"


//#include <ros/ros.h>
//#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "kobuki_robot.h"
#include <iostream>
#include <tf/transform_datatypes.h>


using namespace std;


// Stop motion if we do not receive a command without this timeout
//const ros::Duration COMMAND_TIMEOUT(0.3);

KobukiRobot robot;
volatile double tv = 0, rv = 0;
//ros::Time last_update;

/*
void commandVelCallback(const geometry_msgs::TwistConstPtr twist){
  tv = twist->linear.x;
  rv = twist->angular.z;
  robot.setSpeed(tv, rv);
  robot.sendControls();
  last_update = ros::Time::now();
}
void commandVelStampedCallback(const geometry_msgs::TwistStampedConstPtr twist){
  tv = twist->twist.linear.x;
  rv = twist->twist.angular.z;
  robot.setSpeed(tv, rv);
  robot.sendControls();
  last_update = twist->header.stamp;
  ros::Time current = ros::Time::now();
  ROS_INFO_STREAM("Command received " << (current - last_update) << " after sending");
}
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
int kobuki_main(int argc, char* argv[]) // name must match with APPNAME in Makefile
#endif
  {

   (void) argc;
   (void) argv;
   rclc_init(1,"");
   printf("Turtlebot2 embedded kobuki driver\n");
   rclc_node_t* node = NULL;
   if (node = rclc_create_node("free_kobuki_node", ""))
    {
	//defines publishers and subscribers
	rclc_publisher_t  * pub_odom                = NULL;
	rclc_publisher_t  * pub_imu                 = NULL;
        rclc_subscriber_t * sub_cmd_vel             = NULL;
        rclc_subscriber_t * sub_cmd_vel_stamped     = NULL;

	//defines status variables for creation of publishers and subscribers
	bool pub_odom_ok             = false;
	bool pub_imu_ok              = false;
	bool sub_cmd_vel             = false;
	bool sub_cmd_vel_stamped     = false;
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

        if(sub_cmd_vel = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                                        "cmd_vel", commandVelCallback, 1, false))
        {
	  sub_cmd_vel_ok = true;
	  printf("Created subscriber: cmd_vel.\n");
        } else {
	  print("Failed to create subscriber: cmd_vel.\n");
	}

        if(sub_cmd_vel = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
                                                        "cmd_vel_stamped", commandVeliStampedCallback, 1, false))
        {
          sub_cmd_vel_stamped_ok = true;
          printf("Created subscriber: cmd_vel_stamped.\n");
        } else {
          print("Failed to create subscriber: cmd_vel_stamped.\n");
        }

	//check correct initialization, otherwise abort
        if ( (pub_odom_ok    == false) || (pub_imu_ok     == false) || 
             (sub_cmd_vel_ok == false) || (sub_cmd_vel_ok == false)    )
        {
	  printf("Abort: could not create subscriber or publisher.\n");
	  return 1;
        } else {

	  // connections are properly setup
          std::string serial_device;
  	  std::string odom_topic;
          std::string odom_frame_id;
  	  std::string command_vel_topic;
          std::string imu_frame_id;

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
          odom_frame_id     = "odom";
          imu_frame_id      = "gyro_link";
 
  	  // cerr << "running with params: ";
	  // cerr << "serial_device: " << serial_device << endl;
	  // cerr << "odom_topic: " << odom_topic << endl;
	  // cerr << "odom_frame_id: " << odom_frame_id << endl;
	  // cerr << "imu_frame_id: " << imu_frame_id << endl;
	  // cerr << "command_vel_topic: " << command_vel_topic << endl;

          robot.connect(serial_device);

	  //ros::Rate r(100);
  	  nav_msgs__msg__Odometry odom;
	  nav_msgs__msg__Odemetry__init(&odom);

  	  sensor_msgs__msg__Imu  imu;
          sensor_msgs__msg__Imu__init(&imu);

  	  imu.header.frame_id = imu_frame_id;
  	  odom.header.frame_id = odom_frame_id;
  	  int seq = 0;
  	  int packet_count = 0;

  	  while(true){ // ros::ok() did not work on Olimex with micro-ROS
    		//ros::spinOnce();
    		//ros::Time timestamp;
    		robot.receiveData(timestamp);

		if (packet_count < robot.packetCount()) {
		  // send the odometry
      		  double x,y,theta, vx, vtheta;
      		  robot.getOdometry(x,y,theta,vx,vtheta);
      		  odom.header.seq = seq;
      		  odom.header.stamp = timestamp;
      		  odom.header.frame_id = "base_link";
      		  odom.pose.pose.position.x = x;
      		  odom.pose.pose.position.y = y;
      		  odom.pose.pose.position.z = 0;
      		  double s = sin (theta/2);
      		  double c = cos (theta/2);
      		  odom.pose.pose.orientation.x = 0;
      		  odom.pose.pose.orientation.y = 0;
      		  odom.pose.pose.orientation.z = s;
      		  odom.pose.pose.orientation.w = c;
      		  odom.twist.twist.linear.x = vx;
      		  odom.twist.twist.angular.z = vtheta;
      			
		  rclc_publish( odom_publisher, (const void *) &odom);
		  printf("Sending odom\n");
	
      		  // imu data
      		  double heading;
      		  robot.getImu(heading, vtheta);
		  imu.header.seq = seq;
      		  imu.header.stamp = timestamp;
      		  imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading);
      		  imu.angular_velocity.z = vtheta;
      			
		  rclc_publish( imu_pub, (const void *) &imu); 
		  printf("Sending imu\n");

      		  seq++;
      		  packet_count = robot.packetCount();
    	        }

	  //spin once
	  rclc_spin_node_once(node, 500);
          }
        }        
   }
   return 0;
 } // main

} // extern "C"

/*

  robot.connect(serial_device);

  ros::NodeHandle cmd_nh("~");
  ros::CallbackQueue cmd_queue;
  cmd_nh.setCallbackQueue(&cmd_queue);

  ros::Subscriber command_vel_subscriber =
    cmd_nh.subscribe<geometry_msgs::TwistConstPtr>(
      command_vel_topic, 1,
      &commandVelCallback,
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber command_vel_stamped_subscriber =
    cmd_nh.subscribe<geometry_msgs::TwistStampedConstPtr>(
      "/cmd_vel_stamped", 1,
      &commandVelStampedCallback,
      ros::TransportHints().tcpNoDelay());
  ros::Timer safestop = cmd_nh.createTimer(ros::Duration(.1), &safeStop);

  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

  ros::AsyncSpinner cmd_spinner(1, &cmd_queue);
  cmd_spinner.start();

  ros::Rate r(100);
  nav_msgs::Odometry odom;
  sensor_msgs::Imu imu;
  imu.header.frame_id = imu_frame_id;
  odom.header.frame_id = odom_frame_id;
  int seq = 0;
  int packet_count = 0;

  while(ros::ok()){
    ros::spinOnce();
    ros::Time timestamp;
    robot.receiveData(timestamp);

    if (packet_count < robot.packetCount()) {
      // send the odometry
      double x,y,theta, vx, vtheta;
      robot.getOdometry(x,y,theta,vx,vtheta);
      odom.header.seq = seq;
      odom.header.stamp = timestamp;
      odom.header.frame_id = "base_link";
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0;
      double s = sin (theta/2);
      double c = cos (theta/2);
      odom.pose.pose.orientation.x = 0;
      odom.pose.pose.orientation.y = 0;
      odom.pose.pose.orientation.z = s;
      odom.pose.pose.orientation.w = c;
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.angular.z = vtheta;
      odom_publisher.publish(odom);

      // imu data
      double heading;
      robot.getImu(heading, vtheta);
      imu.header.seq = seq;
      imu.header.stamp = timestamp;
      imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading);
      imu.angular_velocity.z = vtheta;
      imu_pub.publish(imu);

      seq++;
      packet_count = robot.packetCount();
    }

    //r.sleep();  // read already blocks
  }
  */
