#ifndef __JSPUBLISHER_JOYSTICK_H__
#define __JSPUBLISHER_JOYSTICK_H__



#define JOYSTICK_AGENT_INET6_ADDR   "fe80::12e2:d5ff:ff00:1fa"      // micro-ROS agent IPv6 address
#define JOYSTICK_AGENT_UDP_PORT     "9999"                          // UDP port agent is listening on; below 10000
#define RMW_CLIENT_KEY              0xBA5EBA13                      // RMW session client key
#define JOYSTICK_NODE               "uct_joystick"                  // node name max 39 char
#define JOYSTICK_TOPIC              "velocity_cmd"                  // topic name max 39 char
#define PUBLISHER_LOOP_DELAY_MS     50                              // loop delay in milliseconds
#define WATCHDOG_TIME_SEC           3                               // watchdog expired time in seconds
#define JOYSTICK_DEV	            "/dev/js0"                      // joystick device

#endif /* __JSPUBLISHER_JOYSTICK_H__ */

