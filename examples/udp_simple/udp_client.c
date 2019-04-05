/****************************************************************************
 * examples/udp/udp_client.c
 *
 *   Copyright (C) 2007, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "config.h"

#include <sys/types.h>
#include <sys/socket.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>



#include <arpa/inet.h>
#include <netinet/in.h>

#include "udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int create_socket(void)
{
  socklen_t addrlen;
  int sockfd;

  struct sockaddr_in6 addr;

  /* Create a new IPv6 UDP socket */

  sockfd = socket(AF_INET6, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      printf("client ERROR: client socket failure %d\n", errno);
      return -1;
    }

  /* Bind the UDP socket to a IPv6 port */

  addr.sin6_family     = AF_INET6;
  addr.sin6_port       = HTONS(CONFIG_EXAMPLES_UDP_SIMPLE_CLIENT_PORTNO);
  memset(addr.sin6_addr.s6_addr, 0, sizeof(struct in6_addr));
  addrlen              = sizeof(struct sockaddr_in6);

  if (bind(sockfd, (FAR struct sockaddr *)&addr, addrlen) < 0)
    {
      printf("client ERROR: Bind failure: %d\n", errno);
      return -1;
    }

  return sockfd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void udp_client(void)
{

  struct sockaddr_in6 server;

  unsigned char outbuf[SENDSIZE];
  socklen_t addrlen;
  int sockfd;
  int nbytes;
  int offset;


  /* Create a new UDP socket */

  sockfd = create_socket();
  if (sockfd < 0)
    {
      printf("client ERROR: create_socket failed %d\n");
      exit(1);
    }

  //Copy the server configuration
  server.sin6_family            = AF_INET6;
  server.sin6_port              = HTONS(CONFIG_EXAMPLES_UDP_SIMPLE_SERVER_PORTNO);
  memcpy(server.sin6_addr.s6_addr16, g_udpserver_ipv6, 8 * sizeof(uint16_t));
  addrlen                       = sizeof(struct sockaddr_in6);

  //Message to be send
  int i=0;
  char aux_buf[256]="Hello World\n";//Text to be send
  memset(outbuf,' ',SENDSIZE);
  while(aux_buf[i]!='\n'){
    //from the auxilary buffer to the output buffer
    outbuf[i]=aux_buf[i];
    i++;
  }

  printf("Client sending %i characters: ", i);
  printf(aux_buf);
  printf("\n");
  nbytes = sendto(sockfd, outbuf, 12, 0,
                  (struct sockaddr*)&server, addrlen);
  printf("%i bytes send\n",nbytes);
  if (nbytes < 0)
    {
      printf("client: %d. sendto failed: %d\n", offset, errno);
      close(sockfd);
      exit(-1);
    }
  else if (nbytes != SENDSIZE)
    {
      printf("client: %d. Bad send length: %d Expected: %d\n",
             offset, nbytes, SENDSIZE);
      close(sockfd);
      exit(-1);
    }


  close(sockfd);
}
