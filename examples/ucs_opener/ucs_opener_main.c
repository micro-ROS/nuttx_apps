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

#include "init_opener_6lowpan.h"
#include "opener.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_HEARTBEAT		(0x00)

struct relay_ctl_stat {
    int timer;
    bool open;
    bool close;
    char* pstr;
};

char open_str[] = {"OPEN"};
char close_str[] = {"CLOSE"};
char invalid_str[] = {"INVALID"};

int find_opener_command(int cmd, struct relay_ctl_stat* st)
{
    int delay = OPENER_DELAY * 1000 / SUBSCRIBER_PERIOD_MS + 1;
    if(cmd == OPEN_CMD) {
        st->open = ON;
        st->close = OFF;
        st->timer = delay;
        st->pstr = open_str;
    }
    else if(cmd == CLOSE_CMD) {
        st->open = OFF;
        st->close = ON;
        st->timer = delay;
        st->pstr = close_str;
    }
    else {
        st->pstr = invalid_str;
    }
    return 0;
}

int opener_status(struct relay_ctl_stat* st)
{
    return ((int)st->open | (int)st->close << 1);
}

int opener_ctl(int fr0, int fr1, struct relay_ctl_stat* st)
{
    if(fr0 < 0 || fr1 < 0) {
        printf("Failed to open gpio dev \n");
        return -1;
    }
    if(st->timer > 0) {
        st->timer--;
    }
    else {
        st->open = OFF;
        st->close = OFF;        
    }
    ioctl(fr0, GPIOC_WRITE, (unsigned long)st->open);
    ioctl(fr1, GPIOC_WRITE, (unsigned long)st->close);
    printf(" Write %s %s, timer %d \n", st->close == ON ? "ON" : "OFF", \
                                        st->open == ON ? "ON" : "OFF", \
                                        st->timer);
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

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int ucs_opener_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];
    char topic_name2[40];
    int fr0;                            // open relay descriptor
    int fr1;                            // close relay descriptor
    struct relay_ctl_stat ctl_st;       // relay control status
    size_t index;
    std_msgs__msg__Int8 msg;
    msg.data = 0;

    // Opening the relays with read/write permission
    fr0 = open("/dev/gpout0", O_RDWR);
    if (fr0 < 0) {
        printf("Error opening opener relay %d \n", fr0);
        return 0;
    }    
    fr1 = open("/dev/gpout1", O_RDWR);
    if (fr0 < 0) {
        printf("Error opening opener relay %d \n", fr1);
        return 0;
    }    

    if(strlen(OPENER_AGENT_INET6_ADDR) > 39 || strlen(OPENER_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(OPENER_NODE) > 39 || strlen(OPENER_TOPIC) > 39 || strlen(OPENER_TOPIC2) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Initialize relay control 
    ctl_st.open = OFF;
    ctl_st.close = OFF;
    ctl_st.timer = 0;
    ctl_st.pstr = invalid_str;
    // ioctl(fr0, GPIOC_WRITE, (unsigned long)ON);
    // usleep(1000000);    
    // ioctl(fr0, GPIOC_WRITE, (unsigned long)OFF);

    // Initialize 6lowpan
    init_opener_6lowpan();

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, OPENER_AGENT_UDP_PORT);
    strcpy(inet6_address, OPENER_AGENT_INET6_ADDR);
    strcpy(node_name, OPENER_NODE);
    strcpy(topic_name, OPENER_TOPIC);
    strcpy(topic_name2, OPENER_TOPIC2);
    printf("device ID - %d, nOde - %s, topic - %s \n", OPENER_DEVICE_ID, node_name, topic_name );

    rcl_ret_t rv;
    rcl_ret_t rv2;
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

    printf("micro-ROS Opener Subscriber \r\n");

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
	    // RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_PERIOD_MS)));
	    rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_PERIOD_MS));
	    if (wait_set.subscriptions[index]) {
	        rcl_ret_t rv = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
	        if (RCL_RET_OK == rv) {
                find_opener_command(msg.data, &ctl_st);
                printf("Received: Command %s  \n", ctl_st.pstr);
	        }
	    }
    	led_toggle();
        opener_ctl( fr0, fr1, &ctl_st);
	    if (RCL_RET_OK == rv) {
            msg.data = opener_status(&ctl_st);    
            rv = rcl_publish(&publisher, (const void*)&msg, NULL);
            if (RCL_RET_OK == rv ) {
                printf("Sent: '%i'\n", msg.data);
            }
        }

	} while ( RCL_RET_OK == rv );
    
    printf("Error [rcl_take, rcl_publish]: rv = %d \n", rv);

    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    // RCSOFTCHECK(rcl_subscriber_fini(&subscription, &node));
    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}

