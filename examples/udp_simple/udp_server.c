/****************************************************************************
 * examples/udp/udp_server.c
 *
 *   Copyright (C) 2007, 2009, 2012, 2015 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>

#include "udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/



/****************************************************************************
 * Public Functions
 ****************************************************************************/

void udp_server(void)
{

  struct sockaddr_in6 server;
  struct sockaddr_in6 client;

  unsigned char inbuf[1024];
  socklen_t addrlen;
  socklen_t recvlen;
  int sockfd;
  int nbytes;
  int optval;
  int offset;


  /* Create a new UDP socket */

  sockfd = socket(PF_INETX, SOCK_DGRAM, 0);
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

#ifdef CONFIG_EXAMPLES_UDP_SIMPLE_BROADCAST
  optval = 1;
  ret = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(int));
  if (ret < 0)
    {
      printf("Failed to set SO_BROADCAST\n");
      exit(1);
    }
#endif

  /* Bind the socket to a local address */

  server.sin6_family     = AF_INET6;
  server.sin6_port       = HTONS(CONFIG_EXAMPLES_UDP_SIMPLE_SERVER_PORTNO);
  memset(&server.sin6_addr, 0, sizeof(struct in6_addr));

  addrlen                = sizeof(struct sockaddr_in6);


  if (bind(sockfd, (struct sockaddr*)&server, addrlen) < 0)
    {
      printf("server: bind failure: %d\n", errno);
      exit(1);
    }

  //Receiving loop
  recvlen = addrlen;
  printf("Starting UDP Server IPV6\n");
  while(1){
    nbytes = recvfrom(sockfd, inbuf, 1024, 0,
                      (struct sockaddr*)&client, &recvlen);
    printf("Received %d bytes from %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x port %d\n", nbytes,
           client.sin6_addr.s6_addr16[0], client.sin6_addr.s6_addr16[1],
           client.sin6_addr.s6_addr16[2], client.sin6_addr.s6_addr16[3],
           client.sin6_addr.s6_addr16[4], client.sin6_addr.s6_addr16[5],
           client.sin6_addr.s6_addr16[6], client.sin6_addr.s6_addr16[7],
           ntohs(client.sin6_port));
    printf("Received packet: ");
    printf(inbuf);
    printf("\n");
  }

  close(sockfd);
}
