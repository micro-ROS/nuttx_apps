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

#include "kobuki_robot.h"
#include "kobuki_protocol.h"


//NOTE1: need to configure (/root/nuttx$>make menuconfig) standard math library with
//      CONFIG_LIBM=y
//NOTE2: architecture specific math.h does not exist
//      CONFIG_ARCH_MATH=y ==> does not work, because arch/*/math.h does not exist

//#define _USE_MATH_DEFINES
//#include <cmath>
#include <math.h>
//hack above does not work to include M_PI
//#define M_PI       3.1415926535897932384626433832795029


#include <stdexcept>
#include <iostream>
#include "serial.h"
//#include <ros/console.h> 
#define ROS_DEBUG(...)  printf(__VA_ARGS__);

using namespace std;

namespace {
  const float wrap_angle(float angle) {
    if (angle > (float)M_PI) {
      return -(2*(float)M_PI) + angle;
    } else if(angle < -(float)M_PI) {
      return (2*(float)M_PI) - angle;
    } else {
      return angle;
    }
  }
}

KobukiRobot::KobukiRobot() :
_velocity_x(0),
_velocity_theta(0),
_initial_heading(0),
_heading(0) {
  _x = 0;
  _y = 0;
  _theta = 0;
  _left_ticks_per_m = 1./11724.41658029856624751591;
  _right_ticks_per_m = 1./11724.41658029856624751591;
  _baseline = 0.230;
  _first_round = true;
  _serial_fd=-1;
  _parser = new PacketParser();
  _control_packet = new Packet();
}


void KobukiRobot::playSequence(uint8_t sequence) {
  SoundSequencePayload* sp = new SoundSequencePayload;
  sp->sequence = sequence;
  _control_packet->_payloads.push_back(sp);
}

void KobukiRobot::playSound(uint8_t duration, uint16_t note) {
  SoundPayload* sp = new SoundPayload;
  sp->note  = note;
  sp->duration = duration;
  _control_packet->_payloads.push_back(sp);
}

void KobukiRobot::setSpeed(float tv, float rv) {
  BaseControlPayload* bp = new BaseControlPayload;
  // convert to mm;
  tv *=1000;
  float b2 = _baseline * 500;

  if (fabs(tv) < 1){
    //cerr << "pure rotation" << endl;
    bp->radius = 1;
    bp->speed =  (int16_t) (rv * b2);
  } else if (fabs(rv) < 1e-3 ) {
    //cerr << "pure translation" << endl;
    bp->speed = (int16_t) tv;
    bp->radius = 0;
  } else {
    //cerr << "translation and rotation" << endl;
    float r = tv/rv;
    bp->radius = (int16_t) r;
    if (r>1) {
      bp->speed = (int16_t) (tv * (r + b2)/ r);
    } else if (r<-1) {
      bp->speed = (int16_t) (tv * (r - b2)/ r);
    }
  }
  _control_packet->_payloads.push_back(bp);
  ROS_DEBUG("Prepared speed command tv=%.2f, rv=%.2f at pos %ld\n", tv, rv,
    _control_packet->_payloads.size());
}

//TODO time
//void KobukiRobot::receiveData(ros::Time& timestamp) {
void KobukiRobot::receiveData(struct timespec& timestamp) {  
  if (_serial_fd < 0)
    throw std::runtime_error("robot not connected");

  char buf [255];
  int n = read (_serial_fd, buf, 255);
  if(n > 0) {
    clock_gettime(CLOCK_REALTIME, &timestamp);
  }
  for (int i = 0; i<n; i++){
    _sync_finder.putChar(buf[i]);
    if (_sync_finder.packetReady()){
      const unsigned char* b=_sync_finder.buffer();
      Packet* packet = _parser->parseBuffer(b, _sync_finder.bufferLength());
      if (packet) {
        _packet_count++;
        processPacket(packet);
        delete packet;
      }
    }
  }
}

void KobukiRobot::sendControls() {
  if (_serial_fd < 0)
    throw std::runtime_error("robot not connected");

  if (_control_packet->_payloads.size()) {
    unsigned char tx_buffer[1024];
	  int k = _control_packet->write(tx_buffer);
	  if ( k>0 ) {
	    int sent = 0;
	    do {
	      sent += write(_serial_fd, tx_buffer+sent, k-sent);
	    } while (sent<k);
	  }
    ROS_DEBUG("Sent %ld pending control packets\n",
      _control_packet->_payloads.size());

	  _control_packet->clear();
	}
}

void KobukiRobot::connect(std::string device) {
  if (_serial_fd>-1)
    disconnect();
  _serial_fd = serial_open(device.c_str());
  if (_serial_fd<0)
    throw std::runtime_error("error in opening serial port");
  int att = serial_set_interface_attribs (_serial_fd, B115200, 0);
  if (att<0) {
    throw std::runtime_error("error in setting attributes to serial port");
  }
  _packet_count = 0;
}

void KobukiRobot::runFromFile(istream& is) {
  while(is) {
    unsigned char c = is.get();
    _sync_finder.putChar(c);
    if (_sync_finder.packetReady()){
      const unsigned char* b=_sync_finder.buffer();
      Packet* packet = _parser->parseBuffer(b, _sync_finder.bufferLength());
      if (packet) {
        processPacket(packet);
	_packet_count++;
        delete packet;
      }
    }
  }
}

void KobukiRobot::disconnect() {
  if (_serial_fd>-1)
    close(_serial_fd);
  _serial_fd=-1;
}

void KobukiRobot::getOdometry(float& x, float& y, float& theta, float& vx,
  float& vtheta) const {
  x= _x;
  y= _y;
  theta = _theta;
  vx = _velocity_x;
  vtheta = _velocity_theta;
}
void KobukiRobot::getImu(float& heading, float& vtheta) {
  heading = _heading;
  vtheta = _velocity_theta;
}

bool KobukiRobot::bumper(Side s) const {
  switch(s){
  case Right: return _bumper&0x1;
  case Center: return _bumper&0x2;
  case Left: return _bumper&0x4;
  }
  return 0;
}

bool KobukiRobot::cliff(Side s) const {
  switch(s){
  case Right: return _cliff&0x1;
  case Center: return _cliff&0x2;
  case Left: return _cliff&0x4;
  }
  return 0;
}

bool KobukiRobot::wheelDrop(Side s) const {
  switch(s){
  case Right: return _wheel_drop&0x1;
  case Left: return _wheel_drop&0x2;
  case Center: return -1; // NOT POSSIBLE
  }
  return 0;
}

int KobukiRobot::pwm(Side s) const {
  switch(s){
  case Right: return _right_pwm;
  case Left: return _left_pwm;
  case Center: return -1; // NOT POSSIBLE
  }
  return 0;
}

bool KobukiRobot::button(int num) const {
  switch(num){
  case 0: return _button&0x1;
  case 1: return _button&0x2;
  case 2: return _button&0x4;
  }
  return 0;
}

bool KobukiRobot::charger() const {
  return _charger;
}

float KobukiRobot::battery() const {
  return 10.0f*_battery/16.7;
}
float KobukiRobot::voltage() const {
  return _battery/10.0f;
}

bool KobukiRobot::overcurrent(Side s) const {
  switch(s){
  case Right: return _overcurrent_flags&0x1;
  case Left: return _overcurrent_flags&0x2;
  case Center: return -1; // NOT POSSIBLE
  }
  return 0;
}

float KobukiRobot::current(Side s) const {
  switch(s){
  case Right: return _right_motor_current*0.01;
  case Left: return _left_motor_current*0.01;
  case Center: return -1; // NOT POSSIBLE
  }
  return 0;
}

int KobukiRobot::cliffData(Side s) const {
  switch(s){
  case Right: return _right_cliff_signal;
  case Center: return _center_cliff_signal;
  case Left: return _left_cliff_signal;
  }
  return 0;
}

int KobukiRobot::dockingData(Side s) const {
  switch(s){
  case Right: return _right_docking_signal;
  case Center: return _center_docking_signal;
  case Left: return _left_docking_signal;
  }
  return 0;
}

float KobukiRobot::analogGPIO(int channel) const {
  return _analog_input[channel]*3.3/4095;
}

uint16_t KobukiRobot::digitalGPIO() const {
  return _digital_input;
}

void KobukiRobot::processOdometry(uint16_t left_encoder_,
  uint16_t right_encoder_, int32_t elapsed_time_ms){
  if (!_first_round) {
    float dl= _left_ticks_per_m * (left_encoder_-_left_encoder);
    float dr= _right_ticks_per_m * (right_encoder_-_right_encoder);
    float dx = 0, dy = 0, dtheta = (dr-dl)/_baseline;
    if (dl!=dr) {
      float R=.5f*(dr+dl)/dtheta;
      dx = R*sinf((float)dtheta);
      dy = R*(1-cosf((float) dtheta));
    } else {
      dx = dr;
    }
    float s = sinf((float)_theta), c = cosf((float)_theta);
    float diff_x = c * dx - s * dy;
    //ROS_INFO_STREAM("Elapsed " << elapsed_time_ms << " distance " << diff_x);
    _velocity_x = diff_x / (static_cast<float>(elapsed_time_ms) / 1000.0f);
    _x += diff_x;
    _y += s * dx + c * dy;
    _theta += dtheta;
    _theta = fmod(_theta+4*(float)M_PI, 2*(float)M_PI);
    if (_theta>(float)M_PI)
      _theta -= 2*(float)M_PI;
  } else {
    _x = _y = _theta = 0;
  }
  _left_encoder = left_encoder_;
  _right_encoder = right_encoder_;
  //ROS_DEBUG("left=%hu, right=%hu, x=%.4f, y=%.4f\n", left_encoder_, right_encoder_, _x, _y);
}

void KobukiRobot::processPacket(Packet* p) {
  for (size_t i = 0; i<p->_payloads.size(); i++){
    SubPayload *bp = p->_payloads[i];
    switch(bp->header()) {
      // BaseSensorDataPayload:
    case BasicSensorDataPayload_HEADER: {
      BasicSensorDataPayload* bsdp = static_cast<BasicSensorDataPayload*>(bp);
      int32_t elapsed_time = 0;
      // get elapsed time with roll-over handling
      if(bsdp->timestamp > _timestamp) {
        elapsed_time = static_cast<int32_t>(bsdp->timestamp) -
          static_cast<int32_t>(_timestamp);
      } else {
        elapsed_time = 65535 - static_cast<int32_t>(_timestamp) +
          static_cast<int32_t>(bsdp->timestamp);
      }
      _timestamp = bsdp->timestamp;
      _wheel_drop = bsdp->wheel_drop;
      _bumper = bsdp->bumper;
      _cliff = bsdp->cliff;
      _left_pwm = bsdp->left_pwm;
      _right_pwm =bsdp->right_pwm;
      _button = bsdp->button;
      _charger = bsdp->charger;
      _battery = bsdp->battery;
      _overcurrent_flags = bsdp->overcurrent_flags;
      processOdometry(bsdp->left_encoder, bsdp->right_encoder, elapsed_time);
    }
      break;
    case DockingIRPayload_HEADER: {
      DockingIRPayload* dirdp = static_cast<DockingIRPayload*>(bp);
      _right_docking_signal = dirdp->right_signal;
      _center_docking_signal = dirdp->center_signal;
      _left_docking_signal = dirdp->left_signal;
    }
      break;

    case InertialSensorDataPayload_HEADER: {
      InertialSensorDataPayload* idp = static_cast<InertialSensorDataPayload*>(bp);
      inertial_rate = idp->rate;
      inertial_angle = idp->angle;
      // convert to standard ROS representations
      _heading = (static_cast<float>(inertial_angle) / 100.0f) * (M_PI /
        180.0f);
      if(_first_round) {
        _initial_heading = wrap_angle(_heading);
        _heading = 0;
      } else {
        _heading = wrap_angle(_heading - _initial_heading);
      }
      //ROS_DEBUG("Inertial rate %d, heading %d\n", inertial_rate, inertial_angle);
      _velocity_theta = (static_cast<float>(inertial_rate) / 100.0f) * (M_PI
        / 180.0f);
    }
      break;
    case CliffSensorDataPayload_HEADER: {
      CliffSensorDataPayload* csdp = static_cast<CliffSensorDataPayload*>(bp);
      _right_cliff_signal = csdp->right_signal;
      _center_cliff_signal = csdp->center_signal;
      _left_cliff_signal = csdp->left_signal;
    }
      break;
    case CurrentPayload_HEADER: {
      CurrentPayload* cp = static_cast<CurrentPayload*>(bp);
      _right_motor_current = cp->right_motor;
      _left_motor_current = cp->left_motor;
    }
      break;
    case HardwareVersionPayload_HEADER: {
      HardwareVersionPayload* hwp = static_cast<HardwareVersionPayload*>(bp);
      _hw_patch = hwp->patch;
      _hw_major = hwp->major;
      _hw_minor = hwp->minor;
    }
      break;
    case FirmwareVersionPayload_HEADER: {
      FirmwareVersionPayload* fwp = static_cast<FirmwareVersionPayload*>(bp);
      _fw_patch = fwp->patch;
      _fw_major = fwp->major;
      _fw_minor = fwp->minor;
    }
      break;
    case GyroPayload_HEADER: break;
    case GPIOPayload_HEADER: {
      GPIOPayload* gpiop = static_cast<GPIOPayload*>(bp);
      for (int k=0; i<4; i++)
	_analog_input[k] = gpiop->analog_input[k];
      _digital_input = gpiop->digital_input;
    }
      break;
    case UUIDPayload_HEADER: {
      UUIDPayload* uuidp = static_cast<UUIDPayload*>(bp);
      for (int k=0; i<4; i++)
	_udid[k] = uuidp->udid[k];
    }
      break;
    case ControllerInfoPayload_HEADER: {
      ControllerInfoPayload* cip = static_cast<ControllerInfoPayload*>(bp);
      _P=cip->P;
      _I=cip->I;
      _D=cip->D;
    }
      break;
    default:
      ;
    }
  }
  _first_round = false;
}
