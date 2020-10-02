#ifndef __UCS_EFFECTOR_INIT_EFFECTOR_6LOWPAN_H__
#define __UCS_EFFECTOR_INIT_EFFECTOR_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define EFFECTOR_CHANNEL     11          // 6lowpan channel in range 11-26
    #define EFFECTOR_DEVICE_ID   1           // in range 0-255
    #define EFFECTOR_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_effector_6lowpan(void);

#endif /* __UCS_EFFECTOR_INIT_EFFECTOR_6LOWPAN_H__ */

