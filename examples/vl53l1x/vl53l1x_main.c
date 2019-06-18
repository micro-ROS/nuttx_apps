/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <nuttx/sensors/vl53l1x.h>
#include <fcntl.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int vl53l1x_main(int argc, char *argv[])
#endif
{
  int fd; //HIH6130 file descriptor
  uint16_t distancia;
//struct hih6130_s sample;//Data structure of the sensor

//Opening the sensor with read only permission
fd=open("/dev/tof0", O_RDONLY);
if(fd<0){
  printf("Error opening HIH6130 sensor %d\n",fd_temp);
}
ioctl(fd,SNIOC_DISTANCESHORT);
ioctl(fd,SNIOC_CALIBRATE,200);
while(1){
  //Read the data from the sensor and save in the structure
  read(fd&distancia,sizeof(distancia));
  printf("Distancia %i mm \r\n",distancia);
  //Show the data and wait 1 second
  //printf("Temperature: %d ºC Humidity: %d  \n",sample.temp,sample.hum);
  usleep(100000);
}
}
