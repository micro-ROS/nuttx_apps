#ifndef __UCS_EFFECTOR_EFFECTOR_H__
#define __UCS_EFFECTOR_EFFECTOR_H__



#define EFFECTOR_AGENT_INET6_ADDR   "fe80::12e2:d5ff:ff00:1fa"      // micro-ROS agent IPv6 address
#define EFFECTOR_AGENT_UDP_PORT     "9999"                          // UDP port agent is listening on; below 10000
#define EFFECTOR_NODE               "use_case_effector"             // node name max 39 char
#define EFFECTOR_TOPIC              "final_effector_cmd"            // topic name max 39 char
#define EFFECTOR_TOPIC2             "final_effector_status"         // topic name max 39 char
#define LAMP_ON_CMD                 1                               // command 'turn ON the final effector' 
#define LAMP_OFF_CMD                0                               // command 'turn OFF the final effector' 
#define SUBSCRIBER_LOOP_DELAY_MS    180                             // loop delay in milliseconds
#define ON                          1                               // output logic level in state ON
#define OFF                         0                               // output logic level in state OFF


#endif /* __UCS_EFFECTOR_EFFECTOR_H__ */
 
