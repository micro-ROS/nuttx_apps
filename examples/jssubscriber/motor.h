#ifndef __JSSUBSCRIBER_MOTOR_H__
#define __JSSUBSCRIBER_MOTOR_H__



#define MOTOR_AGENT_INET6_ADDR      "fe80::12e2:d5ff:ff00:1fa"   // micro-ROS agent IPv6 address
#define MOTOR_AGENT_UDP_PORT        "9999"                       // UDP port agent is listening on; below 10000
#define RMW_CLIENT_KEY              0xBA5EBA11                   // RMW session client key
#define MOTOR_NODE                  "uct_motor"                  // node name max 39 char
#define MOTOR_TOPIC                 "velocity_cmd"               // topic name max 39 char
#define MOTOR_TOPIC2                "velocity_status"            // topic name max 39 char
#define SUBSCRIBER_LOOP_DELAY_MS    10                           // loop delay in milliseconds
#define WATCHDOG_TIME_SEC           4                            // watchdog expired time in seconds

#endif /* __JSSUBSCRIBER_MOTOR_H__ */
 
