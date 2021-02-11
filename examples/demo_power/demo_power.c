#include <nuttx/config.h>
#include <nuttx/wdog.h>
#include <nuttx/board.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>         
#include <unistd.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_microros/api.h>
// #include <std_msgs/msg/int32.h>

#include <demo_msgs/msg/demo_power.h>
#include <nuttx/sensors/ina219.h>

#include "power.h"

void soft_reset(void);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); soft_reset();}}

#define LED_HEARTBEAT		(0x00)
#define OLIMEX_INA219 "/dev/ina219_1"
#define RPI_INA219 "/dev/ina219_2"

static int try_open(char *dev) {
	int fd =  open(dev, O_RDONLY);

	if (fd < 0) {
		printf("Could not open %s\n", dev);
	}

	return fd;
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
int demo_power_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet_address[40];
    char node_name[40];
    char topic_name[40];
    char topic_name2[40];
	struct ina219_s ina;
	int fd_rpi, fd_olimex;
	float powermw;
	int ret;

	if(strlen(POWER_AGENT_INET_ADDR) > 39 || strlen(POWER_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(POWER_NODE) > 39 || strlen(POWER_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, POWER_AGENT_UDP_PORT);
    strcpy(inet_address, POWER_AGENT_INET_ADDR);
    strcpy(node_name, POWER_NODE);
    strcpy(topic_name, POWER_TOPIC);
    strcpy(topic_name2, POWER_TOPIC2);

    // Create watchdog
    WDOG_ID wdog = wd_create ();
    if( wdog != NULL) {
        printf(" Watchdog created \n");
        wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
    }
    // WDOG_ID wdog = (WDOG_ID)NULL;

    // Opening a sensor device 
	if ((fd_rpi = try_open(RPI_INA219)) < 0 ) {
		return 0;
	}

	if ((fd_olimex = try_open(OLIMEX_INA219)) < 0 ) {
		return 0;
	}

    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()));

    // Set the IP and the port of the Agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    rmw_uros_options_set_udp_address(inet_address, udp_port, rmw_options);

    // Set RMW client key
    rmw_uros_options_set_client_key(RMW_CLIENT_KEY, rmw_options);

    rcl_context_t context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, NULL, &options, &context));

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rcl_node_init(&node, node_name, "", &context, &node_ops));

    printf("micro-ROS Publisher\r\n");
usleep(1000000);

    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(demo_msgs, msg, DemoPower), topic_name, &publisher_ops));

    rcl_publisher_t publisher2 = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(&publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(demo_msgs, msg, DemoPower), topic_name2, &publisher_ops));

    printf(" Power pub_main \n");
	usleep(1000000);
	demo_msgs__msg__DemoPower msg;
	demo_msgs__msg__DemoPower msg2;

    do {
        rv = -1;
		memset(&ina, 0, sizeof(ina));
		ret = read(fd_olimex, &ina, sizeof(ina));
		if (ret == sizeof(ina)) {
			powermw = ((float) ina.voltage) * ((float) ina.current) * 0.000001f;
			if (powermw < 0.0f) {
				powermw = 0;
			}
        		printf("Sending OLI power: %d\n", (int)powermw);
	    	msg.power = powermw;
            rv = rcl_publish(&publisher, (const void*)&msg, NULL);
		}

        if (RCL_RET_OK == rv ) {
			memset(&ina, 0, sizeof(ina));
			ret = read(fd_rpi, &ina, sizeof(ina));
			if (ret == sizeof(ina)) {
				powermw = ((float) ina.voltage) * ((float) ina.current) * 0.000001f;
				if (powermw < 0.0f) {
					powermw = 0;
				}
			    printf("Sending RPI power: %d\n", (int)powermw);
			    msg2.power = powermw;
                rv = rcl_publish(&publisher2, (const void*)&msg2, NULL);
			}
        }
    		
		led_toggle();
		usleep(1000 * PUBLISHER_LOOP_DELAY_MS);
        if( wdog != NULL) {
            wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
        }        
    } while (RCL_RET_OK == rv);
    printf("[rcl_publish]rv %d \n", rv);

    rv = rcl_publisher_fini(&publisher, &node);
    rv = rcl_node_fini(&node);

    printf("\r\nClosing Micro-ROS app\r\n");
    fflush(stdout);
    soft_reset();
    return 0;
}