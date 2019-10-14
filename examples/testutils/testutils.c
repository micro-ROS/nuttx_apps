#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <termios.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <netdb.h>
#include <arpa/inet.h>

#include "netutils/esp8266.h"

void ap_cb(lesp_ap_t *ap)
{
}

#define DEVICE_FILENAME "/dev/ttyS0"

int i =0;
int ret =0;
bool end =true;
struct termios options;
void process_command(){
  char buffer[1024];  /* Input buffer */
  char *bufptr;      /* Current char in buffer */
  int  nbytes =1;       /* Number of bytes read */
  int  tries;        /* Number of tries so far */
  memset(buffer, 0, sizeof(buffer)); //clear command
  int fd = open(DEVICE_FILENAME, O_RDWR);
  if(fd == -1) {
    printf("Could not open %s\n", DEVICE_FILENAME);
    return -1;
  }
  char command[100];
  memset(command, 0, sizeof(command)); //clear command
  printf("Enter the command \n");
  scanf("%s",command);
  strcat(command,"\r\n");
  printf("The command entered is %s \n",command);
  ret = write(fd, command, sizeof(command));
  //ret = write(fd, "AT+CIFSR\r\n", 11);
  printf("written : %d \n",ret);
  int nread=0;
  printf("flag before while loop is %d \n",end);
  while (end)
  {
    nbytes=read(fd,buffer+nread,100);
    nread +=nbytes;
    //printf("number of bytes read is %d \n", nread);
    if (buffer[nread-1] == 'K' && buffer[nread-2] == 'O'){
      //printf("entered in the if loop \n");
      //printf("%c \n",buffer[nread-1]);
      buffer[nread+3]='\0';
      end=false;
      break;
    }
  }
  //printf("flag after while loop is %d \n",end);
  printf("%s",buffer);
}
#ifdef BUILD_MODULE
int main(int argc, char *argv[])
#else
int testutils_main(int argc, char *argv[])
#endif
{
  int choice =0;
  do
  {
    puts("Enter 1 . Command 2. Exit ");
    printf("Enter your choice: ");
    scanf("%d",&choice);
    switch (choice)
    {
    case 1:
      printf("entered case 1 \n");
      process_command();
      end=true;
      break;
    case 2:
      printf("exit \n");
      break;
    default:
      break;
    }
  } while (choice!=2);
  //printf("flag at the end of program is  %d \n",end);
  //close(fd);
  return 0;
}