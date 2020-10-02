#ifndef __UCS_DISTANCE_INIT_DISTANCE_6LOWPAN_H__
#define __UCS_DISTANCE_INIT_DISTANCE_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define DISTANCE_CHANNEL     11          // 6lowpan channel in range 11-26
    #define DISTANCE_DEVICE_ID   3           // in range 0-255
    #define DISTANCE_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_distance_6lowpan(void);

#endif /* __UCS_DISTANCE_INIT_DISTANCE_6LOWPAN_H__ */

