/****************************************************************************
 * examples/telemetry/telemetry_main.c
 *
 *   Copyright (C) 2018 Acutronics Link Robotics. All rights reserved.
 *   Author: Acutronics Link Robotics <juan@erlerobotics.com>
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

#include <fcntl.h>
#include <math.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Variables
 ****************************************************************************/

int a = 0;
float e = 0;

char cpuload_buf[256];
int aux_c;
int lSize;
char *buffer;

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int telemetry_main(int argc, char *argv[])
#endif
{
  int i = 0;
  printf("Mounting file system \n\n");
  system("mount -t procfs /proc");

  FILE *fd_cpu;
  fd_cpu = fopen("/proc/cpuload", "r");
  if (fd_cpu < 0) return -1;
  sleep(1);

  printf("Pushing the CPU to the limits\n");
  printf("We will do the next floating operation: \n");
  printf("e=3.141592653589793\n");
  printf("e=e*e\n");
  printf("This operation 100000 times\n");
  //   sleep(5);
  e = 3.141592653589793;

  while (i < 100000) {
    e = e * e;
    printf("=");
    i++;
  }
  printf("\n");
  printf("\nMax CPU load achieve: \n");
  system("cat /proc/cpuload");

  printf("\nCPU performace test finis!\n");

  printf("Showing interruptions\n\n");
  system("cat /proc/irqs");
  sleep(2);

  printf("Memory ussage\n\n");
  system("cat /proc/meminfo");
  sleep(2);

  printf("Stack of the application\n\n");
  system("cat /proc/self/stack");
  sleep(2);

  printf("Status of the running task\n\n");
  system("cat /proc/self/status");
  sleep(2);

  printf("Type cat /proc/self/status\n");
  printf("To see the context change after running this app\n");
  return 0;

  fclose(fd_cpu);
}
