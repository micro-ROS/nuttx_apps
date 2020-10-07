#ifndef __UCS_OPENER_INIT_OPENER_6LOWPAN_H__
#define __UCS_OPENER_INIT_OPENER_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define OPENER_CHANNEL     11          // 6lowpan radio channel in range 11-26
    #define OPENER_DEVICE_ID   2           // in range 0-255
    #define OPENER_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_opener_6lowpan(void);

#endif /*  __UCS_OPENER_INIT_OPENER_6LOWPAN_H__ */

