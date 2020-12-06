#ifndef __JSSUBSCRIBER_INIT_MTR_6LOWPAN_H__
#define __JSSUBSCRIBER_INIT_MTR_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define MOTOR_CHANNEL     11          // 6lowpan channel in range 11-26
    #define MOTOR_DEVICE_ID   1           // in range 0-255
    #define MOTOR_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_mtr_6lowpan(void);

#endif /* __JSSUBSCRIBER_INIT_MTR_6LOWPAN_H__ */

