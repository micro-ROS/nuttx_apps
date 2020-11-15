/****************************************************************************
 * examples/hidkbd/hidkbd_main.c
 *
 *   Copyright (C) 2011, 2013-2015, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include <nuttx/usb/usbhost.h>
#include <nuttx/input/joystick.h>

#ifdef BUILD_MODULE
int main(int argc, FAR char *argv[])
#else
int js_main(int argc, char *argv[])
#endif
{
	ssize_t nbytes = 0;
	int fd;
	struct joystick_buttonstate_s js_btn;
	memset(&js_btn, 0, sizeof(js_btn));
	char *ptr_js_btn = (char *) &js_btn;

	fd = open("/dev/js0", O_RDONLY);
	if (fd < 0)
	{
		printf("Failed: %d\n", errno);
		fflush(stdout);
		sleep(3);
	}

	for (;;)
	{
		/* Open the keyboard device.  Loop until the device is successfully
		 * opened.
		 */

		do {
			printf("Opening device %s\n", "/dev/js0");
		} while (fd < 0);

		printf("Device %s opened\n", "/dev/js0");
		fflush(stdout);

		/* Loop until there is a read failure (or EOF?) */

		do {
			/* Read a buffer of data */
			nbytes += read(fd, &ptr_js_btn[nbytes], sizeof(js_btn) - nbytes);
			if (nbytes == sizeof(js_btn))
			{
				/* On success, echo the buffer to stdout */

				printf("Joystick info:\n");
				printf("\t - X Axis %d\n", (uint8_t)js_btn.joy_x);
				printf("\t - Y Axis %d\n", (uint8_t)js_btn.joy_y);
				printf("\t - dir button %d\n", js_btn.dir_button);
				printf("\t - Z Axis %d\n", (uint8_t)js_btn.joy_z);
				printf("\t - Unknwon 0 %d\n", js_btn.unknown_0);
				printf("\t - Throttle %d\n", js_btn.throttle);
				printf("\t - Unknwon 1 %d\n", js_btn.unknown_1);
				nbytes = 0;
			}
			sleep(1);
		} while (nbytes > 0);

	}
	printf("Closing device %s: %d\n", "/dev/js0", (int)nbytes);
	fflush(stdout);
	close(fd);

	return 0;
}
