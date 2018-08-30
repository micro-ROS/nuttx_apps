/****************************************************************************
 * examples/udp/udpecho.c
 *
 *   Copyright (C) 2018 Erle Robotics. All rights reserved.
 *   Author: Erle Robotics <juan@erlerobotics.com>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void udpecho_main(void)
{

  struct sockaddr_in server;
  struct sockaddr_in client;
  struct sockaddr_in addr;
  in_addr_t tmpaddr;

  unsigned char inbuf[1024];
  socklen_t addrlen;
  socklen_t recvlen;
  int sockfd;
  int nbytes;
  int optval;
  int offset;


  /* Create a new UDP socket */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      printf("server: socket failure: %d\n", errno);
      exit(1);
    }

  /* Set socket to reuse address */

  optval = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      printf("server: setsockopt SO_REUSEADDR failure: %d\n", errno);
      exit(1);
    }



  /* Bind the socket to a local address */


  server.sin_family      = AF_INET;
  server.sin_port        = HTONS(80); //Change to the port that you want
  server.sin_addr.s_addr = HTONL(INADDR_ANY);

  addrlen                = sizeof(struct sockaddr_in);


  if (bind(sockfd, (struct sockaddr*)&server, addrlen) < 0)
    {
      printf("server: bind failure: %d\n", errno);
      exit(1);
    }

  /* Then receive up to 256 packets of data */

  for (offset = 0; offset < 256; offset++)
    {
      printf("server: %d. Receiving up 1024 bytes\n", offset);
      recvlen = addrlen;
      nbytes = recvfrom(sockfd, inbuf, 1024, 0,
                        (struct sockaddr*)&client, &recvlen);


      tmpaddr = ntohl(client.sin_addr.s_addr);
      printf("server: %d. Received %d bytes from %d.%d.%d.%d:%d\n",
             offset, nbytes,
             tmpaddr >> 24, (tmpaddr >> 16) & 0xff,
             (tmpaddr >> 8) & 0xff, tmpaddr & 0xff,
             ntohs(client.sin_port));

      if (nbytes < 0)
        {
          printf("server: %d. recv failed: %d\n", offset, nbytes);
          //close(sockfd);
          exit(-1);
        }
      else{

        printf("client: %d. Sending %d bytes\n", offset, nbytes);
        nbytes = sendto(sockfd, inbuf, nbytes, 0,
                        (struct sockaddr*)&client, addrlen);
        printf("client: %d. Sent %d bytes\n", offset, nbytes);
      }

    }
  close(sockfd);
}
