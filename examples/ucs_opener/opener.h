#ifndef __UCS_OPENER_OPENER_H__
#define __UCS_OPENER_OPENER_H__



#define OPENER_AGENT_INET6_ADDR     "fe80::12e2:d5ff:ff00:1fa"      // micro-ROS agent IPv6 address
#define OPENER_AGENT_UDP_PORT       "9999"                          // UDP port agent is listening on; below 10000
#define RMW_CLIENT_KEY              0xBA5EBA12                      // RMW session client key
#define OPENER_NODE                 "use_case_opener"               // node name max 39 char
#define OPENER_TOPIC                "opener_cmd"                    // topic name max 39 char
#define OPENER_TOPIC2               "opener_status"                 // topic name max 39 char
#define OPEN_CMD                    1                               // command 'open the door'
#define CLOSE_CMD                   2                               // command 'close the door'
#define ON                          1                               // output logic level in state ON
#define OFF                         0                               // output logic level in state OFF
#define SUBSCRIBER_LOOP_DELAY_MS    180                             // loop delay in milliseconds
#define OPENER_DELAY                22                              // switching OFF/ON delay in seconds
#define WATCHDOG_TIME_SEC           5                               // watchdog expired time in seconds


#endif /* __UCS_OPENER_OPENER_H__ */

