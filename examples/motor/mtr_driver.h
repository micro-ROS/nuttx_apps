/****************************************************************************
 * examples/motor/mtr_driver.h
 *
 * The header file of
 * Tupek demo RoboClaw Motor Controller driver.
 * 
 ****************************************************************************/

#ifndef __MTR_DRIVER_H
#define __MTR_DRIVER_H

#define MOTOR_DEVICE    "/dev/ttyS1"

struct vel_ctrl
{
  short int vel1;
  short int vel2;
};

extern int motor_ctl(const char *);
extern int motor_ctl1(const struct vel_ctrl *);

#endif /* __MTR_DRIVER_H */
