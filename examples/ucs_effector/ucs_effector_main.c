#include <nuttx/config.h>
#include <nuttx/wdog.h>
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

void soft_reset(void);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); soft_reset();}}
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_HEARTBEAT		(0x00)

struct effector_ctl_stat {
    bool cmd_en;
    bool lamp_cmd;
    bool lamp_stat;
    char* pstr;
};

char turn_on_str[] = {"TURN_ON"};
char turn_off_str[] = {"TURN_OFF"};
char invalid_str[] = {"INVALID"};

int find_effector_command(int cmd, struct effector_ctl_stat* st)
{
    if(cmd == LAMP_ON_CMD) {
        st->lamp_cmd = ON;
        st->pstr = turn_on_str;
    }
    else if(cmd == LAMP_OFF_CMD) {
        st->lamp_cmd = OFF;
        st->pstr = turn_off_str;
    }
    else {
        st->pstr = invalid_str;
    }
    st->cmd_en = true;
    return 0;
}

int effector_status(int fd, struct effector_ctl_stat* st)
{
    int ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&st->lamp_stat));
    if (ret < 0) {
        printf("ERROR: Failed to read lamp status \n");
        return 0x80;
    }  
    return (int)st->lamp_stat;
}
int effector_ctl(int fd, struct effector_ctl_stat* st)
{
    if(!st->cmd_en) {
        return 0;
    }
    int ret = ioctl(fd, GPIOC_WRITE, (unsigned long)st->lamp_cmd);
    if(ret < 0) {
        printf("ERROR: Failed to write the lamp command \n");
    }
    else {
        printf(" Write %s \n", st->lamp_cmd == ON ? "ON" : "OFF");
    }
    st->cmd_en = false;
    return 0;    
}

static void led_toggle(void) {
	static int status = 0;
	static int half_seconds = 0;

	if (half_seconds == 5) {
		half_seconds = 0;
		if (status) {
			status = 0;
  			board_autoled_off(LED_HEARTBEAT);
		} else {
			status = 1;
  			board_autoled_on(LED_HEARTBEAT);
		}
	}

	half_seconds++;
}

void soft_reset(void)
{
#if defined(CONFIG_NSH_ROMFSETC) && defined(CONFIG_BOARDCTL_RESET) 
    fflush(stdout);
    usleep(100000);
    board_reset(0);
#endif
}

void wdog_handler(void) 
{
    printf("Watchdog reboots \r\n\n");
    soft_reset();
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
    char topic_name2[40];
    int fr0;                            // device descriptor
    struct effector_ctl_stat ctl_st;    // control status
    size_t index;
    std_msgs__msg__Int8 msg;
    msg.data = 0;

    // Create watchdog
    WDOG_ID wdog = wd_create ();
    if( wdog != NULL) {
        printf(" Watchdog created \n");
        wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
    }

    // Opening the dout device 
    fr0 = open("/dev/gpout0", O_RDWR);
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

    // Initialize effector control structure
    ctl_st.cmd_en = false;
    ctl_st.lamp_cmd = OFF;
    ctl_st.lamp_stat = OFF;
    ctl_st.pstr = invalid_str;

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, EFFECTOR_AGENT_UDP_PORT);
    strcpy(inet6_address, EFFECTOR_AGENT_INET6_ADDR);
    strcpy(node_name, EFFECTOR_NODE);
    strcpy(topic_name, EFFECTOR_TOPIC);
    strcpy(topic_name2, EFFECTOR_TOPIC2);
    
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    printf("device ID - %d, nOde - %s, topic - %s \n", EFFECTOR_PAN_ID, node_name, topic_name );
    // Initialize 6lowpan when running on nsh prompt
    init_effector_6lowpan();
#endif

    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()));

    // Set the IP and the port of the Agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    rmw_uros_options_set_udp_address(inet6_address, udp_port, rmw_options);
    rcl_context_t context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, NULL, &options, &context));

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rcl_node_init(&node, node_name, "", &context, &node_ops));

    printf("micro-ROS Effector Subscriber \r\n");

    // Status publisher
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name2, &publisher_ops));

    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name, &subscription_ops));

	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
	RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()));

	do {
	    RCSOFTCHECK(rcl_wait_set_clear(&wait_set));
	    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index));
	    // RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS)));
	    rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS));
	    if (wait_set.subscriptions[index]) {
	        rv = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
	        if (RCL_RET_OK == rv) {
                find_effector_command(msg.data, &ctl_st);
                printf("Received: Command %s  \n", ctl_st.pstr);
	        }
	    }
    	led_toggle();
        effector_ctl( fr0, &ctl_st);
	    if (RCL_RET_OK == rv) {
            // msg.data = effector_status(&ctl_st);    
            msg.data = effector_status(fr0, &ctl_st);    
            rv = rcl_publish(&publisher, (const void*)&msg, NULL);
            if (RCL_RET_OK == rv ) {
                printf("Effector status: '%i'\n", msg.data);
            }
        }
        if( wdog != NULL) {
            wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
        }

	} while ( RCL_RET_OK == rv );
    printf("Error [rcl_take, rcl_publish]: rv = %d \n", rv);

    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    // RCSOFTCHECK(rcl_subscriber_fini(&subscription, &node));
    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("\r\nClosing Micro-ROS 6lowpan app\r\n");
    soft_reset();
    return 0;
}