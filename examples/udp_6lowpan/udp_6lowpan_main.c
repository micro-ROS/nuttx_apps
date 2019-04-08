/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>


/****************************************************************************
 * Public Functions
 ****************************************************************************/
int udp_write();
void udp_read();

/****************************************************************************
 * Variables
 ****************************************************************************/

uint16_t g_udpserver_ipv6[8] =
{
  HTONS(0),
  HTONS(0),
  HTONS(0),
  HTONS(0),
  HTONS(0),
  HTONS(0),
  HTONS(0),
  HTONS(0)
};

/****************************************************************************
 * Main Functions
 ****************************************************************************/
int udp_6lowpan_main(int argc, char *argv[]){
  if(argc < 2){
    //Mostrar el eror
    printf("Bad ussage \r\n");
    return 0;
  }

  if(strcmp(argv[1], "write") == 0){
    udp_write();
  }
  else if(strcmp(argv[1], "read") == 0){
    udp_read();
  }
  else{
    printf("Error\r\n");
    return 0;
  }
}

int udp_write(){
  //Get IP and destination port
  struct sockaddr_in6 server;

  unsigned char outbuf[256];
  socklen_t addrlen;
  int sockfd;
  int nbytes;
  int offset;


  //Buffers
  char buffer[256];//Buffer to save the income data
  char ip_buffer[256];
  char port_origin[10],port_destination[10];
  int aux=0;
  uint8_t char_rec=0;

  //Getting IPV6 destination
  printf("Introduce the IVP6 Destination\r\n");
  char_rec=scanf("%255s",ip_buffer);
  if(char_rec==0){
    printf("Error: Wrong IP\r\n");
    return 0;
  }
  inet_pton(AF_INET6, ip_buffer, g_udpserver_ipv6);

  //Getting port destination
  printf("Introduce the port destination\r\n");
  char_rec=scanf("%10s",port_destination);
  if(char_rec==0){
    printf("Error: Wrong IP\r\n");
    return 0;
  }

  printf("Introduce the port origin\r\n");
  char_rec=scanf("%10s",port_origin);
  if(char_rec==0){
    printf("Error: Wrong IP\r\n");
    return 0;
  }

  printf("Conection data: \r\n -Dest_IP: %s \r\n -Dest_Port: %s\r\n -Origin_Port: %s\r\n",ip_buffer,port_destination,port_origin);

/**********************Creating client connection***************************/

  //Creating socket
  sockfd = socket(AF_INET6, SOCK_DGRAM, 0);
  if (sockfd < 0){
    printf("Error: Socket creation failure %d\n", errno);
    return 0;
  }

  /* Bind the UDP socket to a IPv6 port */

  server.sin6_family     = AF_INET6;
  server.sin6_port       = HTONS(8080);
  memset(server.sin6_addr.s6_addr, 0, sizeof(struct in6_addr));
  addrlen              = sizeof(struct sockaddr_in6);

  if (bind(sockfd, (FAR struct sockaddr *)&server, addrlen) < 0){
      printf("Error: Bind failure: %d\n", errno);
      return -1;
  }

/**********************Creating client connection***************************/

  //Copy the server configuration
  server.sin6_family            = AF_INET6;
  server.sin6_port              = HTONS(61616);
  memcpy(server.sin6_addr.s6_addr16, g_udpserver_ipv6, 8 * sizeof(uint16_t));
  addrlen                       = sizeof(struct sockaddr_in6);

  //Message to be send
  while(1){
    char buffer[256];
    printf("Introduce a message to send:\r\n");
    char_rec=scanf("%255s",buffer);
    if(strcmp("quit",buffer)==0) return 0;
    printf("Sending %i characters: %s \r\n",char_rec,buffer);
    sendto(sockfd, buffer, 4, 0,
                    (struct sockaddr*)&server, addrlen);
  }


}

void udp_read(){

}

void lowpan_configuration(){

}
