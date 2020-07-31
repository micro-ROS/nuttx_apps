#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
// #include <nuttx/sensors/hih6130.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
// #include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>

#include "init_pub_6lowpan.h"
#include "pub.h"


int get_command(int cmd)
{
    return (cmd == ON_CMD) ? OFF_CMD : ON_CMD;
}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int ucs_pub_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];

    if(strlen(PUB_AGENT_INET6_ADDR) > 39 || strlen(PUB_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(PUB_NODE) > 39 || strlen(PUB_OPENER_TOPIC) > 39 || strlen(PUB_EFFECTOR_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Initialize 6lowpan
    init_pub_6lowpan();

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, PUB_AGENT_UDP_PORT);
    strcpy(inet6_address, PUB_AGENT_INET6_ADDR);
    // strcpy(node_name, PUB_NODE);
    strcpy(node_name, "UCS_CMD");
    if(CONFIG_UCS_PUB_EXAMPLE_TOPIC_SELECTOR) {
        strcpy(topic_name, PUB_OPENER_TOPIC);
    }
    else {
        strcpy(topic_name, PUB_EFFECTOR_TOPIC);
    }
    printf("device ID - %d, nOde - %s, topic - %s \n", PUB_DEVICE_ID, node_name, topic_name );

    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();

    rv = rcl_init_options_init(&options, rcl_get_default_allocator());
    if (RCL_RET_OK != rv) {
        printf("rcl init options error: %s\n", rcl_get_error_string().str);
        return 1;
    }
    // Set the IP and the port of the Agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    rmw_uros_options_set_udp_address(inet6_address, udp_port, rmw_options);

    rcl_context_t context = rcl_get_zero_initialized_context();
    rv = rcl_init(0, NULL, &options, &context);
    if (RCL_RET_OK != rv) {
        printf("rcl initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    printf("micro-ROS Publisher\r\n");

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    rv = rcl_node_init(&node, node_name, "", &context, &node_ops);
    if (RCL_RET_OK != rv) {
        printf("Node initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name, &publisher_ops);
    if (RCL_RET_OK != rv) {
        printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

        
    printf(" pub_main \n");
	std_msgs__msg__Int8 msg;
    const int num_msg = 5;
    int cmd = OFF_CMD;
    int heartbeats = 0;
    do {
        cmd = get_command(cmd);
        msg.data = cmd;          
        heartbeats++;              
        rv = rcl_publish(&publisher, (const void*)&msg, NULL);
        if (RCL_RET_OK == rv )
        {
            printf("Sent: Command %s  \n", cmd == ON_CMD ? "ON": "OFF");
            usleep(5000000);
        }
    } while (RCL_RET_OK == rv);    
    // } while (RCL_RET_OK == rv);    

    rv = rcl_publisher_fini(&publisher, &node);
    rv = rcl_node_fini(&node);


    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}