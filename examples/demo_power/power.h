#ifndef __UCS_POWER_POWER_H__
#define __UCS_POWER_POWER_H__



#define POWER_AGENT_INET_ADDR       "192.168.10.2"       // micro-ROS agent IPv4 address
#define POWER_AGENT_UDP_PORT        "8888"               // UDP port agent is listening on; below 10000
#define RMW_CLIENT_KEY              0xBA5EBA22           // RMW session client key
#define POWER_NODE                  "power"              // node name max 39 char
#define POWER_TOPIC                 "power_oli"          // topic name max 39 char
#define POWER_TOPIC2                "power_rpi"          // topic name max 39 char
#define PUBLISHER_LOOP_DELAY_MS     180                  // loop delay in milliseconds
#define WATCHDOG_TIME_SEC           5                    // watchdog expired time in seconds


#endif /* __UCS_POWER_POWER_H__ */