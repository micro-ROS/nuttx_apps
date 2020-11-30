/****************************************************************************
 * examples/jsmotor/jsmotor_main.c
 *
 * Motor is controled by Logitech Extreme 3D PRO joystick
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include <nuttx/usb/usbhost.h>
#include <nuttx/input/joystick.h>

#include "js_driver.h"

#define JOYSTICK_DEV	"/dev/js0"

#ifdef BUILD_MODULE
int main(int argc, FAR char *argv[])
#else
int jsmotor_main(int argc, char *argv[])
#endif
{
	int fd;
	struct vel_ctrl vel;

	sleep(1);
	fd = open_js(JOYSTICK_DEV);
	if (fd < 0)
	{
		printf("Failed to open joystick device \n");
		fflush(stdout);
		return 0;
	}

	for (;;)
	{
		js2motor(fd, &vel);
		printf(" %d, %d \n", vel.vel1, vel.vel2);

	}
	close_js(fd);

	return 0;
}

