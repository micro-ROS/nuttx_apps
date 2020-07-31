#ifndef __UCS_PUB_PUB_H__
#define __UCS_PUB_PUB_H__



#define PUB_AGENT_INET6_ADDR      "fe80::12e2:d5ff:ff00:1fa"     // micro-ROS agent IPv6 address
#define PUB_AGENT_UDP_PORT        "9999"                         // UDP port agent is listening on; below 10000
#define PUB_NODE                  "use_case_sensor"              // node name max 39 char
#define PUB_OPENER_TOPIC          "opener_cmd"                   // opener topic name max 39 char
#define PUB_EFFECTOR_TOPIC        "effector_cmd"                 // effector topic name max 39 char
#define ON_CMD                    1                              // command 'open the door' or 'lamp_on'
#define OFF_CMD                   2                              // command 'close the door' or 'lamp off'


#endif /* __UCS_PUB_PUB_H__ */

