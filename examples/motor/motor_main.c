/****************************************************************************
 * examples/motor/motor_main.c
 *
 * TUPEK motor driving test
 * 
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "mtr_driver.h"


#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int motor_main(int argc, char *argv[])
#endif
{
  int ret;
  struct vel_ctrl velctrl;
  if (argc == 2)
  {
    ret = motor_ctl(argv[1]);
  }
  else if (argc == 3)
  {
    velctrl.vel1 = atoi(argv[1]);
    velctrl.vel2 = atoi(argv[2]);
    printf("ctrl %d %d \n", velctrl.vel1, velctrl.vel2);
    ret = motor_ctl1(&velctrl);
  }
  else
  {
    fprintf(stderr, "Usage: %s velocity [velocity]\r\n", argv[0]);
    ret = 0;
  }
  if(ret)
  {
    printf("Driver error %d\r\n", ret);
  }

  return 0;
}
