/****************************************************************************
 * examples/motor/mtr_driver.c
 *
 * The Tupek demo RoboClaw Motor Controller driver
 * to drive a controller in serial mode.
 *  
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "mtr_driver.h"

#define MTR_VELOCITY_LIMIT  0x3f 
#define MTR_STOP            0x00
#define MTR_ZERO_CH1        0x40
#define MTR_ZERO_CH2        0xC0
#define MTR_REVERSE_DIR     1

int mtr_drive(unsigned int dir, unsigned int vel);
int mtr_drive2(unsigned int dir1, unsigned int vel1, unsigned int dir2, int unsigned vel2);

int mtr_send_cmd(unsigned char cmd)
{
  // printf("Velocity cmd = %.2x \r\n", cmd); 
  int ret;
  int fd = open(MOTOR_DEVICE, O_WRONLY);
  if(fd < 0)
  {
    return -2;
  }
  ret =  write(fd,&cmd,1);
  close(fd);
  if(ret != 1) {
    printf(" mtr_driver.c/mtr_send_cmd write failure  \n"); 
  }
  return 0;
}

int mtr_stop_cmd()
{
  unsigned char cmd = MTR_STOP;
  return mtr_send_cmd(cmd);
}

unsigned char mtr_chanel_cmd(unsigned int ch, unsigned int dir, unsigned int vel)
{
  int zero ;
  if(ch == 1)
  {
    zero = MTR_ZERO_CH1;
  }
  else
  {
    zero = MTR_ZERO_CH2;    
  }

  unsigned char v;
  if(!dir)
  {
    v = zero + vel;
  }
  else
  {
    v = zero - vel;
  }
  return v;
}

int mtr_drive(unsigned int dir, unsigned int vel)
{
  return mtr_drive2(dir, vel, dir, vel);
}

int mtr_drive1(unsigned int ch, unsigned int dir, unsigned int vel)
{
  if((dir > MTR_REVERSE_DIR) || (vel > MTR_VELOCITY_LIMIT) || ((ch != 1) && (ch != 2)))
  {
    printf("Invalid velocity!!\r\n");
    return -1;      
  }

  unsigned char cmd = mtr_chanel_cmd(ch, dir, vel);
  // printf("Ch %1d ", ch); 
  return mtr_send_cmd(cmd);
}


int mtr_drive2(unsigned int dir1, unsigned int vel1, unsigned int dir2, int unsigned vel2)
{
  if((dir1 > MTR_REVERSE_DIR) || (dir2 > MTR_REVERSE_DIR) || (vel1 > MTR_VELOCITY_LIMIT) || (vel2 > MTR_VELOCITY_LIMIT))
  {
    printf("Invalid velocity!!\r\n");
    return -1;      
  }

  unsigned int ch = 1;
  int ret = mtr_drive1(ch, dir1, vel1);
  
  ch = 2;  
  return ret | mtr_drive1(ch, dir2, vel2);
}

int motor_ctl(const char* cmd)
{
  int ret;
  int vel1, vel2;
  int num = sscanf( cmd, "%d,%d",  &vel1, &vel2 );
  if(num == 1) {
    int dir1 = 0;
    // printf("velocity = %d\r\n", vel1);
    if(vel1 < 0) {
      vel1 = -vel1;
      dir1 = 1;
    } 
    ret = mtr_drive(dir1, vel1);
  }
  else if(num == 2) {
    int dir1 = 0;
    int dir2 = 0;
    // printf("velocity 1 = %d, ", vel1);
    if(vel1 < 0) {
      vel1 = -vel1;
      dir1 = 1;
    } 
    // printf("velocity 2 = %d\r\n", vel2);
    if(vel2 < 0) {
      vel2 = -vel2;
      dir2 = 1;
    } 
    ret = mtr_drive2(dir1, vel1, dir2, vel2);
  }
  else {
    ret = -1;
  }
  if(ret) {
    printf("Motor driver error %d\r\n", ret);
  }
  return ret;
}

int motor_ctl1(const struct vel_ctrl * ctrl)
{
  int ret;
  int vel1;
  int vel2;
  int dir1;
  int dir2;
  if(ctrl->vel1 < 0) {
    vel1 = -ctrl->vel1;
    dir1 = 1;
  } 
  else {
    vel1 = ctrl->vel1;
    dir1 = 0;
  }
  if(ctrl->vel2 < 0) {
    vel2 = -ctrl->vel2;
    dir2 = 1;
  }
  else {
    vel2 = ctrl->vel2;
    dir2 = 0;
  } 
  //  printf("vel_ctrl : %d, %d,   dir: (%d %d)   vel: (%d %d) \n", ctrl->vel1, ctrl->vel2, dir1, dir2, vel1, vel2);

  return mtr_drive2(dir1, vel1, dir2, vel2);
}



