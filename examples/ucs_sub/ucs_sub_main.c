#include <nuttx/config.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
#include <std_msgs/msg/int32.h>

#include "init_sub_6lowpan.h"
#include "sub.h"


#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int ucs_sub_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];

    if(strlen(SUB_AGENT_INET6_ADDR) > 39 || strlen(SUB_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(SUB_NODE) > 39 || strlen(SUB_DISTANCE_TOPIC) > 39 || strlen(SUB_HIH_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }
    
    // Initialize 6lowpan
    init_sub_6lowpan();

    // Define agent's udp port and IPv6 address, then uros node and topic name. 
    strcpy(udp_port, SUB_AGENT_UDP_PORT);
    strcpy(inet6_address, SUB_AGENT_INET6_ADDR);
    strcpy(node_name, SUB_NODE);
    if(CONFIG_UCS_SUB_EXAMPLE_TOPIC_SELECTOR) {
        strcpy(topic_name, SUB_DISTANCE_TOPIC);
    }
    else {
        strcpy(topic_name, SUB_HIH_TOPIC);
    }
    printf("device ID - %d, nOde - %s, topic - %s \n", SUB_DEVICE_ID, node_name, topic_name );

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

    printf("micro-ROS Subscriber \r\n");

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    // rv = rcl_node_init(&node, "int32_subscriber_rcl", "", &context, &node_ops);
    rv = rcl_node_init(&node, node_name, "", &context, &node_ops);
    if (RCL_RET_OK != rv)
    {
        fprintf(stderr, "[main] error in rcl : %s\n", rcutils_get_error_string().str);
        rcl_reset_error();
        return 1;
    }   

    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rv = rcl_subscription_init(
        &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name, &subscription_ops);
    if (RCL_RET_OK != rv) {
        printf("Subscription initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    rv = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
    if (RCL_RET_OK != rv) {
        printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rv = rcl_wait_set_clear(&wait_set);
    if (RCL_RET_OK != rv) {
        printf("Wait set clear error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    size_t index;
    rv = rcl_wait_set_add_subscription(&wait_set, &subscription, &index);
    if (RCL_RET_OK != rv) {
        printf("Wait set add subscription error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    printf(" UCS sub main \n");
    
    void* msg = rcl_get_default_allocator().zero_allocate(sizeof(std_msgs__msg__Int32), 1, rcl_get_default_allocator().state);
    do {
        rv = rcl_wait(&wait_set, 1000000);
        for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
            rv = rcl_take(wait_set.subscriptions[i], msg, NULL, NULL);
            if (RCL_RET_OK == rv)
            {
                int distance = ((const std_msgs__msg__Int32*)msg)->data;
                printf("Received: Distance %s  \n", distance);
            }
        }
    } while ( RCL_RET_OK == rv);
    printf("Error [rcl_take]: rv = %d \n", rv);

    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}