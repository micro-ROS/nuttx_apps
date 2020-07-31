#ifndef __UCS_SUB_INIT_SUB_6LOWPAN_H__
#define __UCS_SUB_INIT_SUB_6LOWPAN_H__



#define SUB_CHANNEL     11          // 6lowpan radio channel in range 11-26
#define SUB_DEVICE_ID   0           // in range 0-255
#define SUB_PAN_ID      0xabcd      // 6lowpan PAN_ID

int init_sub_6lowpan(void);

#endif /*  __UCS_SUB_INIT_SUB_6LOWPAN_H__ */

