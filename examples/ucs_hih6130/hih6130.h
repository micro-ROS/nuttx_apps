#ifndef __UCS_HIH6130_HIH6130_H__
#define __UCS_HIH6130_HIH6130_H__



#define HIH_AGENT_INET6_ADDR        "fe80::12e2:d5ff:ff00:1fa"          // micro-ROS agent IPv6 address
#define HIH_AGENT_UDP_PORT          "9999"                              // UDP port agent is listening on; below 10000
#define HIH_NODE                    "use_case_hih6130"                  // node name max 39 char
// #define HIH_TOPIC                   "temp_humidity"                     // topic name max 39 char
#define HIH_TOPIC                   "temperature"                       // topic name max 39 char
#define HIH_TOPIC2                  "humidity"                          // topic name max 39 char
#define PUBLISHER_LOOP_DELAY_MS     10                                 // loop delay in milliseconds
#define WATCHDOG_TIME_SEC           5                               // watchdog expired time in seconds


#endif /*  __UCS_HIH6130_HIH6130_H__ */

