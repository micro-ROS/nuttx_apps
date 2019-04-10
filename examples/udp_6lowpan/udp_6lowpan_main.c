/****************************************************************************
 * examples/udp_6lowpan/udp_6lowpan_main.c
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics <juan@erlerobotics.com>
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
#include <string.h>

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int udp_write();
void udp_read();
void lowpan_configuration();

/****************************************************************************
 * Variables
 ****************************************************************************/

/****************************************************************************
 * Main Functions
 ****************************************************************************/
int udp_6lowpan_main(int argc, char *argv[]) {

  printf("Do you want to execute the automatic WPAN configuration? (y/n)\n");
  char aux = getchar();
  if (aux == 'y')
    lowpan_configuration();

  while (1) {
    printf("\nAvailable commands\n -To send a package type: write \n -To "
           "receive a package type: read \n -To exit type: quit\n");
    char buffer[256];
    scanf("%255s", buffer);
    if (strcmp(buffer, "write") == 0) {
      udp_write();
    } else if (strcmp(buffer, "read") == 0) {
      udp_read();
    } else if (strcmp(buffer, "quit") == 0) {
      return 0;
    } else {
      printf("\nWrong command.\n -To send a package type: write \n -To receive "
             "a package type: read \n -To exit type: quit\n");
    }
  }
}
