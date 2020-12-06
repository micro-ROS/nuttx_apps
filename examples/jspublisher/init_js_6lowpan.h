#ifndef __JSPUBLISHER_INIT_JS_6LOWPAN_H__
#define __JSPUBLISHER_INIT_JS_6LOWPAN_H__


// When running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    #define JOYSTICK_CHANNEL     11          // 6lowpan channel in range 11-26
    #define JOYSTICK_DEVICE_ID   3           // in range 0-255
    #define JOYSTICK_PAN_ID      0xabcd      // 6lowpan PAN_ID
#endif

int init_js_6lowpan(void);

#endif /* __JSPUBLISHER_INIT_JS_6LOWPAN_H__ */

