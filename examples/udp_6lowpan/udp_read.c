
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/socket.h>
#include <sys/types.h>

#include <nuttx/config.h>

char port_origin[10];

int optval;
int char_rec;

socklen_t addrlen;
socklen_t recvlen;

struct sockaddr_in6 server;
struct sockaddr_in6 client;

int sockfd;
int nbytes;

int offset;

void udp_read() {

  printf("Introduce the reception port\r\n");
  char_rec = scanf("%10s", port_origin);
  if (char_rec == 0) {
    printf("Error: Wrong port\r\n");
    return 0;
  }
  /* Create a new UDP socket */

  sockfd = socket(PF_INET6, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    printf("server: socket failure: %d\n", errno);
    exit(1);
  }

  optval = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void *)&optval,
                 sizeof(int)) < 0) {
    printf("server: setsockopt SO_REUSEADDR failure: %d\n", errno);
    exit(1);
  }

  /* Bind the socket to a local address */

  server.sin6_family = AF_INET6;
  server.sin6_port = HTONS(atoi(port_origin));

  memset(&server.sin6_addr, 0, sizeof(struct in6_addr));

  addrlen = sizeof(struct sockaddr_in6);

  if (bind(sockfd, (struct sockaddr *)&server, addrlen) < 0) {
    printf("server: bind failure: %d\n", errno);
    exit(1);
  }

  printf("Listening on %s for input packets\n", port_origin);

  sleep(5);
  recvlen = addrlen;
  while (1) {
    unsigned char inbuf[1024];
    memset(&inbuf, 0, 1024);
    nbytes =
        recvfrom(sockfd, inbuf, 1024, 0, (struct sockaddr *)&client, &recvlen);

    printf("Received %d bytes from %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x "
           "port %d\n",
           nbytes, client.sin6_addr.s6_addr16[0], client.sin6_addr.s6_addr16[1],
           client.sin6_addr.s6_addr16[2], client.sin6_addr.s6_addr16[3],
           client.sin6_addr.s6_addr16[4], client.sin6_addr.s6_addr16[5],
           client.sin6_addr.s6_addr16[6], client.sin6_addr.s6_addr16[7],
           ntohs(client.sin6_port));
    printf("Received packet: ");
    printf(inbuf);
    printf("\n");

    usleep(100);
  }
}
