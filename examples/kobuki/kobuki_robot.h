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


#pragma once
#include <stdint.h>
#include <string>
#include <istream>
#include <pthread.h>
#include "kobuki_protocol.h"


class Packet;
class PacketSyncFinder;
class PacketParser;
namespace ros {
  class Time;
}

//TODO replace with
//#include<rcl/time.h>
//rcl_lcokc_ myclock
//rcl_ros_clock_init(myClock, rcutils_get_dfault_allocator);
//rcl_time_point_value myNow;
//rcl_clock_get_now/myClock, myNow);
// __Time myTimeMsg

typedef struct {
  int32_t   sec;
  uint32_t  nsec;
} uros_time_t;
  
class KobukiRobot {
public:
  enum Side {Left, Center, Right};
  KobukiRobot();
  void connect(std::string device);
  void disconnect();
  void runFromFile(std::istream& is);

  //void receiveData(ros::Time&);
  void receiveData(struct timespec &);
  
  void sendControls();
  void playSequence(uint8_t sequence);
  void playSound(uint8_t duration, uint16_t note);
  void setSpeed(float tv, float rv); // tv: meters/s, rv:radians/s
  int packetCount() const {return _packet_count;}


  /**  Accessor ,methods */
  void getOdometry(float& x, float& y, float& theta, float& vx,
    float& vtheta) const;
  void getImu(float & heading, float& vtheta);
  bool bumper(Side s) const;
  bool cliff(Side s) const;
  bool wheelDrop(Side s) const;
  int pwm(Side s) const;   //0-255
  bool button(int num) const;
  bool charger() const;
  float voltage() const;
  float battery() const;   //percentage
  bool overcurrent(Side s) const ;
  float current(Side s) const ;   //ampere
  int cliffData(Side s) const ;

  /**
    Flag will be setted when signal is detected
    0x01 for NEAR_LEFT state
    0x02 for NEAR_CENTER state
    0x04 for NEAR_RIGHT state
    0x08 for FAR_CENTER state
    ox10 for FAR_LEFT state
    0x20 for FAR_RIGHT state
  */
  int dockingData(Side s) const;
  float analogGPIO(int channel) const ;   // volts, max 3.3
  uint16_t digitalGPIO() const;    // first 4 bits */

  /*

  inline float gyroAngle();
  inline float gyroRate();
  */

public:
  uint16_t _timestamp;
  uint8_t _bumper;
  uint8_t _wheel_drop;
  uint8_t _cliff;
  uint16_t _left_encoder, _right_encoder;
  uint8_t _left_pwm, _right_pwm;
  uint8_t _button;
  uint8_t _charger;
  uint8_t _battery;
  uint8_t _overcurrent_flags;
  uint8_t _right_docking_signal, _center_docking_signal, _left_docking_signal;
  uint16_t _right_cliff_signal, _center_cliff_signal, _left_cliff_signal;
  int16_t inertial_rate, inertial_angle;
  uint8_t _right_motor_current, _left_motor_current;
  uint8_t _hw_patch, _hw_major, _hw_minor;
  uint8_t _fw_patch, _fw_major, _fw_minor;
  uint16_t _analog_input[4];
  uint16_t _digital_input;
  uint32_t _udid[3];
  uint32_t _P,_I,_D;
  float _x, _y, _theta;
  float _velocity_x, _velocity_theta;
  float _initial_heading, _heading;
  float _baseline, _left_ticks_per_m, _right_ticks_per_m;
  bool _first_round;
  int _packet_count;

  void processOdometry(uint16_t left_encoder_, uint16_t right_encoder_,
    int32_t elapsed_time);
  void processPacket(Packet* p);

  int _serial_fd;
  PacketSyncFinder _sync_finder;
  PacketParser* _parser;
  Packet* _currentPacket;
  Packet* _control_packet;
};


//TODO misssing trigonometric functions
//float sinf(float v) { return 0.0;}
//float cosf(flaot v) { return 0.0;}
//double fmod (double x, double y) {return 0;}
//double fabs(double x){return 0;}
