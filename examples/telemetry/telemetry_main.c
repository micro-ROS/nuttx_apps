/****************************************************************************
 * examples/telemetry/telemetry_main.c
 *
 *   Copyright (C) 2018 Juan Flores Muñoz. All rights reserved.
 *   Author: Juan Flores Muñoz <jfloresmu92@gmail.com>
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
 #include <fcntl.h>
 #include <unistd.h>
 #include <nuttx/sensors/ina219.h>


 /****************************************************************************
  * Public Functions
  ****************************************************************************/

 /****************************************************************************
  * telemetry_main
  ****************************************************************************/


 #ifdef CONFIG_BUILD_KERNEL
 int main(int argc, FAR char *argv[])
 #else
 int telemetry_main(int argc, char *argv[])
 #endif
 {
   //Variables
   struct ina219_s sample;
   int fd;
   int ret;
   FILE *fd_f;
   char aux_c;
   int aux_i=0;//Index of the cpu load
   char cpuload[6];
   char meminfo[256];
   //Fist it necessary to mount the proc file system
   system("mount -t procfs /proc");

   //Values of the energetic consumption from the INA219 sensor
   fd = open("/dev/ina219", O_RDWR); //Open the sensor
   if(fd<0){
     printf("Sensor not available\n");
   }
   ret = read(fd, &sample, sizeof(sample));//Reading the value of sensor
   if (ret != sizeof(sample)) return 0;//Checking if the data are correct
   close(fd);//Closing the sensor

   //Getting the load in the CPU.
   //Previously check in the config: RTOS Features->
   //->Performace monitoring ->Enable CPU load monitoring
   FILE *acc = fopen("/proc/cpuload", "r"); //Opening the file of the CPU Load

   //This way of get the values is not the best, but for now it works.
   //I need to find out how to use fscanf
   while(1) {
      aux_c = fgetc(acc);
      if( feof(acc) ) break;
      cpuload[aux_i]=aux_c;
      aux_i++;
   }
   aux_i=0;
   fclose(acc);

   //Getting the load in the CPU.
   //Previously check in the config: RTOS Features->
   //->Performace monitoring ->Enable CPU load monitoring
   acc = fopen("/proc/meminfo", "r"); //Opening the file of the CPU Load

   //This way of get the values is not the best, but for now it works.
   //I need to find out how to use fscanf
   while(1) {
      aux_c = fgetc(acc);
      if( feof(acc) ) break;
      meminfo[aux_i]=aux_c;
      aux_i++;
   }
   aux_i=0;
   fclose(acc);


   int power=((sample.current/1000)*(sample.current/1000)*(sample.voltage/1000))/1000000;
   //Getting the ussage of the SRAM
   printf("CPU Load: %s\n", cpuload);
   printf(meminfo);
   printf("\n");
   printf("Bus Voltage: %4u mV Current: %4u mA\n ",(sample.voltage/1000), (sample.current/1000));
   printf("Power consumption: %4u mW \n",power);

 }
