#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
#include <std_msgs/msg/int8.h>

#include "init_effector_6lowpan.h"
#include "effector.h"

struct effector_ctl_stat {
    bool lamp;
    char* pstr;
};

char turn_on_str[] = {"TURN_ON"};
char turn_off_str[] = {"TURN_OFF"};
char invalid_str[] = {"INVALID"};

int find_effector_command(int cmd, struct effector_ctl_stat* st)
{
    if(cmd == LAMP_ON_CMD) {
        st->lamp = ON;
        st->pstr = &turn_on_str;
    }
    else if(cmd == LAMP_OFF_CMD) {
        st->lamp = OFF;
        st->pstr = &turn_off_str;
    }
    else {
        st->pstr = &invalid_str;
    }
    return 0;
}

int effector_ctl(int fr0, struct effector_ctl_stat* st)
{
    if(fr0 < 0) {
        printf("Failed to open gpio dev \n");
        return -1;
    }
    ioctl(fr0, GPIOC_WRITE, (unsigned long)st->lamp);
    printf(" Write %s \n", st->lamp == ON ? "ON" : "OFF");
    return 0;    
}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int ucs_effector_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];
    int fr0;                            // effector device descriptor
    struct effector_ctl_stat ctl_st;    // effector control status

    // Opening the dout device with write only permission
    fr0 = open("/dev/gpout0", O_WRONLY);
    if (fr0 < 0) {
        printf("Error opening device %d \n", fr0);
        return 0;
    }    

    if(strlen(EFFECTOR_AGENT_INET6_ADDR) > 39 || strlen(EFFECTOR_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(EFFECTOR_NODE) > 39 || strlen(EFFECTOR_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Initialize relay control structure
    ctl_st.lamp = OFF;
    ctl_st.pstr = &invalid_str;

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, EFFECTOR_AGENT_UDP_PORT);
    strcpy(inet6_address, EFFECTOR_AGENT_INET6_ADDR);
    strcpy(node_name, EFFECTOR_NODE);
    strcpy(topic_name, EFFECTOR_TOPIC);
    
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    printf("device ID - %d, nOde - %s, topic - %s \n", EFFECTOR_PAN_ID, node_name, topic_name );
    // Initialize 6lowpan when running on nsh prompt
    init_effector_6lowpan();
#endif

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

    printf("micro-ROS Effector Subscriber \r\n");

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
    // rv = rcl_subscription_init(
    //     &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &subscription_ops);
    rv = rcl_subscription_init(
        &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name, &subscription_ops);
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

    printf(" Effector main \n");
    // ioctl(fr0, GPIOC_WRITE, (unsigned long)ON);
    // usleep(1000000);    
    // ioctl(fr0, GPIOC_WRITE, (unsigned long)OFF);
    
    void* msg = rcl_get_default_allocator().zero_allocate(sizeof(std_msgs__msg__Int8), 1, rcl_get_default_allocator().state);
    do {
        effector_ctl( fr0, &ctl_st);
        rv = rcl_wait(&wait_set, 1000000);
        for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
            rv = rcl_take(wait_set.subscriptions[i], msg, NULL, NULL);
            if (RCL_RET_OK == rv)
            {
                int cmd = ((const std_msgs__msg__Int8*)msg)->data;
                find_effector_command(cmd, &ctl_st);
                printf("Received: Command %s  \n", ctl_st.pstr);
            }
        }
    } while ( RCL_RET_OK == rv || RCL_RET_SUBSCRIPTION_TAKE_FAILED == rv);
    printf("Error [rcl_take]: rv = %d \n", rv);

    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}