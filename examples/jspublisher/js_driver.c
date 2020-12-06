/****************************************************************************
 * examples/jspublisher/js_driver.c
 *
 * The Logitech Extreme 3D PRO joystick driver.
 *  
 * Code to drive a RoboClaw Motor Controller in serial mode.
 * Joystick position is converted into a velocity command:
 *  x ---> velocity
 *  z ---> turning left/right
 * 
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

struct joystick_axes
{
	uint16_t joy_x;
	uint16_t joy_y;
	uint16_t joy_z;
};

struct joystic_axes_descript
{
	uint16_t edgepos1;				// left or down edge position
	uint16_t zero1;					// left or down posision of dead zone
	uint16_t zero2;					// up or right position of dead zone
	uint16_t edgepos2;				// up or right edge position  
};

struct vel_ctrl gvelctrl;
int16_t gvel;
int16_t gvelturn;
int8_t gstatus;
struct joystic_axes_descript js_dscr_x = {JOY_X_MIN, JOY_X_DEAD_MIN, JOY_X_DEAD_MAX, JOY_X_MAX};
struct joystic_axes_descript js_dscr_y = {JOY_Y_MAX, JOY_Y_DEAD_MAX, JOY_Y_DEAD_MIN, JOY_Y_MIN};
struct joystic_axes_descript js_dscr_z = {JOY_Z_MIN, JOY_Z_DEAD_MIN, JOY_Z_DEAD_MAX, JOY_Z_MAX};

void axes_parser(uint8_t *bf, struct joystick_axes *pa)
{
pa->joy_x = (((uint16_t)bf[1] & 0x03) << 8) + (uint16_t)bf[0];
pa->joy_y = ((((uint16_t)bf[2] & 0x0f) << 8) + (uint16_t)bf[1]) >> 2;
pa->joy_z = (uint16_t)bf[3];
}

int16_t get_vel(int16_t pos, struct joystic_axes_descript *pdscr )
{	
	int16_t vel;
	bool dir = pdscr->edgepos2 > pdscr->edgepos1;

	if((dir && (pos < pdscr->zero1)) || (!dir && (pos > pdscr->zero1)))
	{
		vel = -VEL_LIMIT * (pos - pdscr->zero1) / (pdscr->edgepos1 - pdscr->zero1);
	}
	else if((dir && (pos > pdscr->zero2)) || (!dir && (pos < pdscr->zero2)))
	{
		vel = VEL_LIMIT * (pos - pdscr->zero2) / (pdscr->edgepos2 - pdscr->zero2);
	}
	else
	{
		vel = 0;
	}
	
	return vel;
}

void check_limits1(void)
{
  if(gvel > VEL_LIMIT)
    {
      gvel = VEL_LIMIT;
    }
  if(gvelturn > VEL_LIMIT)
    {
      gvelturn = VEL_LIMIT;
    }
}

void check_limits(void)
{
  if(gvelctrl.vel1 > VEL_LIMIT)
    {
      gvelctrl.vel1 = VEL_LIMIT;
    }
  else if(gvelctrl.vel1 < -VEL_LIMIT)
    {
      gvelctrl.vel1 = -VEL_LIMIT;
    }
  if(gvelctrl.vel2 > VEL_LIMIT)
    {
      gvelctrl.vel2 = VEL_LIMIT;
    }
  else if(gvelctrl.vel2 < -VEL_LIMIT)
    {
      gvelctrl.vel2 = -VEL_LIMIT;
    }
}

void find_velctrl(void)
{
  if(gstatus == NORMAL)
  {
    gvelctrl.vel1 = gvel;
    gvelctrl.vel2 = gvel;
  }
  else 
  {
    gvelctrl.vel1 = gvel - gvelturn / 2;
    gvelctrl.vel2 = gvel + gvelturn / 2;
  }
}

void test_print1(int x, int y, int z)
{
  char *pstatus;
  char st_normal[] = "NORMAL";
  char st_left[]   = "LEFT  ";
  char st_right[]  = "RIGHT ";
  char st_none[]   = "NONE  ";
  switch (gstatus)
  {
  case NORMAL:
    pstatus = st_normal;
    break;
  case TURN_LEFT:
    pstatus = st_left;
    break;
  case TURN_RIGHT:
    pstatus = st_right;
    break;
  default:
    pstatus = st_none;
    break;
  }
  printf("(pos x, y, z): %2d, %2d, %d", x, y, z);
  printf(",     (vel, turm, status): % 3d, % 3d, %s", gvel, gvelturn, pstatus);
  printf(",     (ctrl v2, v1): (%2d, %2d) \r", gvelctrl.vel2, gvelctrl.vel1);
  fflush(stdout);
}


void update_jsdriver(uint8_t *bf, struct vel_ctrl *pvel)
{
  	struct joystick_axes  js_axes;

	axes_parser(bf, &js_axes);
	gvel = get_vel(js_axes.joy_y, &js_dscr_y);
	if(JOY_TURN_Z_AXIS)
	{
		gvelturn = get_vel(js_axes.joy_z, &js_dscr_z);
	}
	else
	{
		gvelturn = get_vel(js_axes.joy_x, &js_dscr_x);
	}
  	check_limits1();

	if(gvelturn < 0)
	{
        gstatus = TURN_LEFT;                    
	}
	else if(gvelturn > 0)
	{
        gstatus = TURN_RIGHT;                    
	}
	else
	{
        gstatus = NORMAL;                    
	}
  	find_velctrl();
	check_limits();
	test_print1(js_axes.joy_x, js_axes.joy_y, js_axes.joy_z);
  	*pvel = gvelctrl;				
}

void init_jsdriver(void)
{
     // Initialize velocity parameters
  	gvel = 0;
  	gvelturn = 0;
  	gstatus = NORMAL;
  	find_velctrl();

}

void js2motor(int fd, struct vel_ctrl * pvel)
{
	ssize_t nbytes = 0;
	struct joystick_buttonstate_s js_btn;
	memset(&js_btn, 0, sizeof(js_btn));
	char *ptr_js_btn = (char *) &js_btn;
	uint8_t *bf = (uint8_t *) &js_btn;

		/* Loop until there is a read failure (or EOF?) */
		do {
			/* Read a buffer of data */
			nbytes += read(fd, &ptr_js_btn[nbytes], sizeof(js_btn) - nbytes);
			if (nbytes == sizeof(js_btn))
			{
				update_jsdriver(bf, pvel);
				nbytes = 0;
			}
			usleep(SLEEP_DELAY);
		} while (nbytes > 0);
}

int open_js(const char *pdev)
{
	init_jsdriver();
	return open(pdev, O_RDONLY);
}

int close_js(int fs)
{
	return close(fs);
}


