/****************************************************************************
 * examples/jssubscriber/jssubscriber_main.c
 *
 * The Tupec demo micro-ROS subscriber application.
 * It subscribes velocity commands from the joystick jspublisher
 * to stear a Tupec motor.  
 * 
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wdog.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
#include <std_msgs/msg/u_int32.h>

#include "init_mtr_6lowpan.h"
#include "mtr_driver.h"
#include "motor.h"

#define LOOPS_TO_STOP       10                  // Number of no command loops to STOP
                                                // Tupek motor 

void soft_reset(void);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); soft_reset();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_HEARTBEAT		(0x00)


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
int jssubscriber_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];
    char topic_name2[40];

    int fd;                            // TUPEC_MOTOR device descriptor
    struct vel_ctrl velctrl;
    uint32_t vel_cmd_cnt = 0;

    // struct effector_ctl_stat ctl_st;    // control status

    size_t index;
    // std_msgs__msg__Int8 msg;
    std_msgs__msg__UInt32 msg;
    msg.data = 0;

    // // Create watchdog
    // WDOG_ID wdog = wd_create ();
    // if( wdog != NULL) {
    //     printf(" Watchdog created \n");
    //     wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
    // }
    WDOG_ID wdog = NULL;

	// fd = open(TUPEC_MOTOR, O_RDONLY);
	// if (fd < 0) {
		// printf("Could not open %s\n", TUPEC_MOTOR);
	if (motor_ctl("0") < 0) {
		printf("Error: Could not open the motor device \n");
        return 0;
	}
    if(strlen(MOTOR_AGENT_INET6_ADDR) > 39 || strlen(MOTOR_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(MOTOR_NODE) > 39 || strlen(MOTOR_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, MOTOR_AGENT_UDP_PORT);
    strcpy(inet6_address, MOTOR_AGENT_INET6_ADDR);
    strcpy(node_name, MOTOR_NODE);
    strcpy(topic_name, MOTOR_TOPIC);
    strcpy(topic_name2, MOTOR_TOPIC2);
    
// Initialize 6lowpan when running on nsh prompt
#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    printf("device ID - %d, nOde - %s, topic - %s \n", MOTOR_PAN_ID, node_name, topic_name );
    init_mtr_6lowpan();
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

    printf("micro-ROS Motor Subscriber \r\n");

    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    // subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE ;
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
	// RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_name, &subscription_ops));
	RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), topic_name, &subscription_ops));

	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
	RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()));
    int ret;
	do {
	    RCSOFTCHECK(rcl_wait_set_clear(&wait_set));
	    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index));
	    // RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS)));
	    rcl_wait(&wait_set, RCL_MS_TO_NS(SUBSCRIBER_LOOP_DELAY_MS));
	    if (wait_set.subscriptions[index]) {
	        rv = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
	        if (RCL_RET_OK == rv) {
                // velctrl =  *((struct vel_ctrl*)&((std_msgs__msg__UInt32*)msg)->data);
                velctrl =  *((struct vel_ctrl*)(&(msg.data)));
                // printf(" Received velocity_cmd: v1 %2d, v2 %2d \n", velctrl.vel1, velctrl.vel2);
                vel_cmd_cnt = LOOPS_TO_STOP;
	        }
	    }
    	led_toggle();
        if(!vel_cmd_cnt)
        {
            velctrl.vel1 = 0;
            velctrl.vel2 = 0;
        }
        else
        {
            vel_cmd_cnt--;
        }
        ret = motor_ctl1(&velctrl);
        printf(" Send to motor velocity_cmd: (%d), v1 %2d, v2 %2d \n", ret, velctrl.vel1, velctrl.vel2);

        if( wdog != NULL) {
            wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
        }

	} while ( RCL_RET_OK == rv );
    printf("Error [rcl_take, rcl_publish]: rv = %d \n", rv);

    rv = rcl_subscription_fini(&subscription, &node);
    rv = rcl_node_fini(&node);   

    printf("\r\nClosing Micro-ROS 6lowpan app\r\n");
    soft_reset();
    return 0;
}