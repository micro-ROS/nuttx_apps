

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * telemetry_main
 ****************************************************************************/


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int adc_simple_main(int argc, char *argv[])
#endif
{
  int fd;
  ssize_t nbytes;
  struct adc_msg_s sample[1];

  fd = open("/dev/adc0", O_RDONLY);
  if (fd < 0)
  {
    printf("Error opening the ADC\n");
    return 0;
  }
  size_t readsize = sizeof(struct adc_msg_s);
  while(1){
   read(fd, &sample[1], readsize);
    if(nbytes<0){
      printf("Error reading");
      break;
    }
    printf("%d: channel: %d value: %d\n",
           1, sample[1].am_channel, sample[1].am_data);
  }

  close(fd);

}
