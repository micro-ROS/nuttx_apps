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
 #include <stdlib.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <nuttx/sensors/ina219.h>

 /****************************************************************************
  * Public Functions
  ****************************************************************************/

  /****************************************************************************
   * Variables
   ****************************************************************************/
  struct ina219_s sample;

 /****************************************************************************
  * hello_main
  ****************************************************************************/

 #ifdef CONFIG_BUILD_KERNEL
 int main(int argc, FAR char *argv[])
 #else
 int telemetry_main(int argc, char *argv[])
 #endif
 {
   //File descriptors
   FILE *fd_save;
   FILE *fd_sensor;
   FILE *fd_cpu;
   FILE *fd_mem;

   //Buffers
   char cpuload_buf[6];
   char meminfo_buf[256];
   char buffer[256];

   //Auxilary Variables
   int i=0;
   char aux_c;
   int n=0;
   char aux[10];
   int iterations=0;
   int iterations_counter=0;

   //Checking the arguments of the function.
   if(argc<4){
     //It's neccesary almost the name of the file
     printf("Correct: telemetry file.txt number_measures show_option\n");
     printf("If you want a continue measure, write i as argument \n");
     printf("The available options are:\n s (to save in the sd card)\n");
     printf("c (to see the data in the console) \n b (both modes)\n");
     return -1;
   }
   else if(argc==4){
     //This is to check the name of the file
     sprintf(buffer,"/mnt/%s",argv[1]);
     int num=atoi(argv[2]);
     //To set the number of iterations
     if(num>1 && num<=500){
       iterations=num;
     }
     else if(argv[2]=='i'){
       iterations=-1;
     }
     else{
       printf("Error, must be 1 to 500 iterations or i to continue measurement\n");
       return -1;
     }
    /* if(argv[3]!= 's' || argv[3]!= 'c' || argv[3]!= 'b'){
       printf("Must be, s, c or b\n");
       return -1;
     }*/
   }
   else{
     printf("Too much arguments\n");
     printf("Correct: telemetry file.txt number_measures show_option\n");
     printf("If you want a continue measure, write i as argument \n");
     printf("The available options are:\n s (to save in the sd card)\n");
     printf("c (to see the data in the console) \n b (both modes)\n");
     return -1;
   }



   //Commands to mount the file system of the SD card and the data of the board
   system("mount -t vfat /dev/mmcsd0 /mnt");
   system("mount -t procfs /proc");

   //Opening the Files
   fd_save = fopen( buffer , "a+" );
   fd_cpu = fopen("/proc/cpuload", "r");
   fd_mem = fopen("/proc/meminfo", "r");
   fd_sensor = open("/dev/ina219", O_RDWR);

   if(fd_save < 0 || fd_cpu < 0 || fd_mem < 0|| fd_sensor < 0){
     printf("Error opening file\n");
     return -1;
   }

   //This loop gets the data from the sensor and from the files, then it
   //construct the message, and finally it save in the file
   printf("Starting telemetry SD\n");
   /*buffer="New Measure\n";//This is just in case we use the same file
   fwrite(aux_buffer , 1 , n , fd_save );*/

   while(iterations!=iterations_counter){
     //Reading the value of sensor
     read(fd_sensor, &sample, sizeof(sample));
     //getting the data from the CPU
     while(1){
       aux_c = fgetc(fd_cpu);
       if( feof(fd_cpu) ) break;
       cpuload_buf[i]=aux_c;
       i++;
     }
     i=0;
     //getting the data from the CPU
     while(1){
       aux_c = fgetc(fd_mem);
       if( feof(fd_mem) ) break;
       meminfo_buf[i]=aux_c;
       i++;
     }
     //With this loop, we look for the value of used SRAM
     for(i=0;i<10;i++){
       aux[i]=meminfo_buf[76+i];
     }
     i=127072 - atoi(aux);
     //Creating the message
     n=sprintf(buffer,"V: %4u mV I: %4u mA CPU: %s Free SRAM: %d Bytes\n",
     sample.voltage,sample.current,cpuload_buf,i);
     //Writing to the file
     if(argv[3]=='s'){
       //Save in the SD
       fwrite(buffer , 1 , n , fd_save );
     }
     else if(argv[3]=='c'){
       //Show in the console
       printf(buffer);
     }
     else{
       fwrite(buffer , 1 , n , fd_save );
       printf(buffer);
     }

     usleep(200);
     iterations_counter++;
   }

   //Closing the files
   fclose(fd_save);
   fclose(fd_cpu);
   fclose(fd_mem);
   close(fd_sensor);
   return 0;
 }
