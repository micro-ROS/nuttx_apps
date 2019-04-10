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

#include <nuttx/config.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>

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
  server.sin6_port       = HTONS(atoi(port_origin));
  memset(server.sin6_addr.s6_addr, 0, sizeof(struct in6_addr));
  addrlen              = sizeof(struct sockaddr_in6);

  if (bind(sockfd, (FAR struct sockaddr *)&server, addrlen) < 0){
      printf("Error: Bind failure: %d\n", errno);
      return -1;
  }

/**********************Creating client connection***************************/

  //Copy the server configuration
  server.sin6_family            = AF_INET6;
  server.sin6_port              = HTONS(atoi(port_destination));
  memcpy(server.sin6_addr.s6_addr16, g_udpserver_ipv6, 8 * sizeof(uint16_t));
  addrlen                       = sizeof(struct sockaddr_in6);

  //Message to be send
  while(1){
    char buffer[256];
    printf("Introduce a message to send:\r\n");
    char_rec=scanf("%255s",buffer);
    if(strcmp("quit",buffer)==0){
      printf("Closing connection\n");
      close(sockfd);
      return;
    }
    printf("Sending %i characters: %s \n\n",strlen(buffer),buffer);
    sendto(sockfd, buffer, strlen(buffer), 0,
                    (struct sockaddr*)&server, addrlen);
  }


}
