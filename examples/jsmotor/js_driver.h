/****************************************************************************
 * examples/jsmotor/js_driver.h
 *
 * The Logitech Extreme 3D PRO joystick driver header file.
 *
 ****************************************************************************/
#ifndef __JS_DRIVER_H
#define __JS_DRIVER_H


#define JOY_X_MIN			    0
#define JOY_X_MAX			    1023
#define JOY_X_DEAD_MIN	 	424
#define JOY_X_DEAD_MAX	 	624
#define JOY_Y_MIN			    0
#define JOY_Y_MAX			    1023
#define JOY_Y_DEAD_MIN	 	424
#define JOY_Y_DEAD_MAX	 	624
#define JOY_Z_MIN			    0
#define JOY_Z_MAX			    255
#define JOY_Z_DEAD_MIN	 	103
#define JOY_Z_DEAD_MAX	 	175
#define JOY_TURN_Z_AXIS		true

#define TURN_LEFT         1
#define TURN_RIGHT        2
#define NORMAL            0

#define VEL_LIMIT         63
#define SLEEP_DELAY       1000     // looping delay in usec

struct vel_ctrl
{
  int16_t vel1;       // right side
  int16_t vel2;       // left side
};

extern int open_js(const char *);
extern int close_js(int);
extern void js2motor(int , struct vel_ctrl *);

#endif /* __JS_DRIVER_H */