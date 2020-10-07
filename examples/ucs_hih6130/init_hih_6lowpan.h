#ifndef __UCS_HIH6130_INIT_6LOWPAN_H__
#define __UCS_HIH6130_INIT_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define HIH_CHANNEL     11          // 6lowpan channel in range 11-26
    #define HIH_DEVICE_ID   4           // in range 0-255
    #define HIH_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_hih_6lowpan(void);

#endif /*  __UCS_HIH6130_INIT_6LOWPAN_H__ */

