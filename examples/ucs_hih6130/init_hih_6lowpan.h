#ifndef __UCS_HIH6130_INIT_6LOWPAN_H__
#define __UCS_HIH6130_INIT_6LOWPAN_H__



#define HIH_CHANNEL     11          // 6lowpan channel in range 11-26
#define HIH_DEVICE_ID   2           // in range 0-255
#define HIH_PAN_ID      0xabcd      // 6lowpan PAN_ID

int init_hih_6lowpan(void);

#endif /*  __UCS_HIH6130_INIT_6LOWPAN_H__ */

