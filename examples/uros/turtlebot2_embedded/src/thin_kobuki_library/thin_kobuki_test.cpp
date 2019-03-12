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


#include "serial.h"
#include "kobuki_robot.h"
#include "kobuki_protocol.h"
#include <ros/time.h>
#include <iostream>
#include <linux/joystick.h>

using namespace std;


int main (int argc, char** argv) {

  Packet p;
  BaseControlPayload* bpc = new BaseControlPayload;
  bpc->speed = 100;
  bpc->radius =  4;
  p._payloads.push_back(bpc);
  unsigned char buf[1024];
  p.write(buf);

  int fd = open ("/dev/input/js0", O_RDONLY|O_NONBLOCK);
  if (fd<0) {
    cerr << "no joy found" << endl;
  }

  KobukiRobot r;
  r.connect(argv[1]);
  r.playSequence(0);
  float tv = 0;
  float rv = 0;
  float tvscale = .1/32767.0;
  float rvscale = 1/32767.0;

  while (1) {
    struct js_event e;
    while (read (fd, &e, sizeof(e)) > 0 ){
      int axis = e.number;
      int value = e.value;
      if (axis == 1) {
	tv = -value * tvscale;
      }
      if (axis == 4) {
	rv = -value *rvscale;
      }
    }
    r.setSpeed(tv,rv);
    ros::Time timestamp;
    r.receiveData(timestamp);
    r.sendControls();
    double x,y,theta, vx, vtheta;
    r.getOdometry(x,y,theta,vx,vtheta);
    cerr << "speed: [ " << tv << " " << rv << " ]";
    cerr << " pose:  [ " << x << " " << y << " " << theta << " ]" << endl;
  }
}
