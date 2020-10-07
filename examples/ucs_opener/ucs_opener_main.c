#include <nuttx/config.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
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

void soft_reset(void);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); soft_reset();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_HEARTBEAT		(0x00)

struct relay_ctl_stat {
    bool cmd_en;
    bool open_cmd;
    bool close_cmd;
    bool open_stat;
    bool close_stat;
    bool timer_en;
    uint cmd_tick;
    char* pstr;
};

char open_str[] = {"OPEN"};
char close_str[] = {"CLOSE"};
char invalid_str[] = {"INVALID"};

int check_timer(uint tick)
{
    uint c_tick = clock_systimer();
    if(tick > c_tick) {
        tick = 0xffff - tick + c_tick;
    }
    else {
        tick = c_tick - tick;
    }
    if(tick < TICK_PER_SEC * OPENER_DELAY) {
        printf("  timer: %i s;  ", tick / TICK_PER_SEC);
        return 0;
    }
    return 1;
    
}

int find_opener_command(int cmd, struct relay_ctl_stat* st)
{
    int delay = OPENER_DELAY * 1000 / SUBSCRIBER_LOOP_DELAY_MS + 1;
    if(cmd == OPEN_CMD) {
        st->open_cmd = ON;
        st->close_cmd = OFF;
        st->pstr = open_str;
        st->cmd_tick = clock_systimer();
        st->timer_en = true;
    }
    else if(cmd == CLOSE_CMD) {
        st->open_cmd = OFF;
        st->close_cmd = ON;
        st->pstr = close_str;
        st->cmd_tick = clock_systimer();
        st->timer_en = true;
    }
    else {
        st->pstr = invalid_str;
    }
    st->cmd_en = true;
    return 0;
}

int opener_status(int fr0, int fr1, struct relay_ctl_stat* st)
{
    int ret = ioctl(fr0, GPIOC_READ, (unsigned long)((uintptr_t)&st->open_stat));
    ret |= ioctl(fr1, GPIOC_READ, (unsigned long)((uintptr_t)&st->close_stat));
    if (ret < 0) {
        printf("ERROR: Failed to read status \n");
        return 0x80;
    }    
    return ((int)st->open_stat | (int)st->close_stat << 1);
}

int opener_ctl(int fr0, int fr1, struct relay_ctl_stat* st)
{
    int ret;

    if(st->cmd_en) {
        ret = ioctl(fr0, GPIOC_WRITE, (unsigned long)st->open_cmd);
        ret |= ioctl(fr1, GPIOC_WRITE, (unsigned long)st->close_cmd);
        if(ret) {
            printf("ERROR: Failed to write the open/close command \n");
        }
        else {
            printf(" Write %s %s \n", st->close_cmd == ON ? "ON" : "OFF", \
                                            st->open_cmd == ON ? "ON" : "OFF");
        }
        st->cmd_en = false;
    }

    if(st->timer_en && check_timer(st->cmd_tick)) { 
            st->open_cmd = OFF;
            st->close_cmd = OFF;        
            st->cmd_en = true;
            st->timer_en = false;
            st->cmd_tick = 0;
    }    
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

    // Create watchdog
    WDOG_ID wdog = wd_create ();
    if( wdog != NULL) {
        printf(" Watchdog created \n");
        wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
    }

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
    ctl_st.cmd_en = false;
    ctl_st.open_cmd = OFF;
    ctl_st.close_cmd = OFF;
    ctl_st.open_stat = OFF;
    ctl_st.close_stat = OFF;
    ctl_st.timer_en = false;
    ctl_st.cmd_tick = 0;
    ctl_st.pstr = invalid_str;

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, OPENER_AGENT_UDP_PORT);
    strcpy(inet6_address, OPENER_AGENT_INET6_ADDR);
    strcpy(node_name, OPENER_NODE);
    strcpy(topic_name, OPENER_TOPIC);
    strcpy(topic_name2, OPENER_TOPIC2);

#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    printf("device ID - %d, nOde - %s, topic - %s \n", OPENER_DEVICE_ID, node_name, topic_name );
    // Initialize 6lowpan when running on nsh prompt
    init_opener_6lowpan();
#endif
    
    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()));

    // Set the IP and the port of the Agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    rmw_uros_options_set_udp_address(inet6_address, udp_port, rmw_options);

    // Set RMW client key
    rmw_uros_options_set_client_key(RMW_CLIENT_KEY, rmw_options);

    rcl_context_t context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, NULL, &options, &context));

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rcl_node_init(&node, node_name, "", &context, &node_ops));

    printf("micro-ROS Opener Subscriber \r\n");

    // Status publisher
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    // publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE ;
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name2, &publisher_ops));

    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    // subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE ;
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name, &subscription_ops));

	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
	RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()));

	int rvp = RCL_RET_OK;
	do {
		RCSOFTCHECK(rcl_wait_set_clear(&wait_set));
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index));
		// RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS)));
		usleep(SUBSCRIBER_LOOP_DELAY_MS);
		rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS));
		if (wait_set.subscriptions[index]) {
			rv = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
			if (RCL_RET_OK == rv) {
				find_opener_command(msg.data, &ctl_st);
				printf("Received: Command %s  \n", ctl_st.pstr);
			}
		}
		led_toggle();
		opener_ctl( fr0, fr1, &ctl_st);
		if (RCL_RET_OK == rv) {
			msg.data = opener_status(fr0, fr1, &ctl_st);    
			rvp = rcl_publish(&publisher, (const void*)&msg, NULL);
    			printf("Error : rvs = %d, rvp = %d \n", rv, rvp);
			if (RCL_RET_OK == rvp ) {
				printf("Opener status '%i' \n", msg.data);
			}
		}
		if( wdog != NULL) { 
			wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
		}
	} while ( (RCL_RET_OK == rvp && RCL_RET_OK == rv) || ctl_st.timer_en );
    
    printf("Error [rcl_take, rcl_publish]: rv = %d \n", rv);

    soft_reset();
#if 0
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    // RCSOFTCHECK(rcl_subscriber_fini(&subscription, &node));
    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("\r\nClosing Micro-ROS 6lowpan app\r\n");
#endif
    return 0;
}

