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
#include <std_msgs/msg/int32.h>

#include "init_distance_6lowpan.h"
#include "distance.h"

void soft_reset(void);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); soft_reset();}}

#define OLIMEX_TFMINI                   "/dev/ttyS1"        // on USART6   
#define TFMINI_FRAME_HEADER_BYTE        (0x59)
#define TFMINI_FRAME_SPARE_BYTE         (0x00)
#define LED_HEARTBEAT					(0x00)

struct tfmini_frame {
        uint8_t headers[2];
        uint8_t dist_l;
        uint8_t dist_h;
        uint8_t strength_l;
        uint8_t strength_h;
        uint8_t integr_time;
        uint8_t spare_byte;
        uint8_t checksum;
} __attribute__((packed)) ;

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

static int read_all(int fd, char *data, size_t size)
{
	int readd = 0;
	int n;

	while ((n = read(fd, &data[readd], size - readd)) >= 0) {
		if (readd == size)
			return -1;

		readd += n;
	}

	return readd;
}

static int synchronize(int fd, struct tfmini_frame *tmf)
{
        ssize_t rc;
        unsigned int i=0;
	char data;
       	char *datas;
	unsigned int checksum;

	memset(tmf, 0, sizeof(*tmf));
        while (rc = read(fd, &data, 1) == 1) {

                if (data == TFMINI_FRAME_HEADER_BYTE && !i) {
			checksum = data;
                        i=1;
                } else if (data == TFMINI_FRAME_HEADER_BYTE && i) {
			checksum += data;
                        break;
                } else {
                        i=0;
		}
        }

	if (rc < 0) {
		return rc;
	}

	if ((rc = read_all(fd, (char *) &tmf->dist_l, sizeof(struct tfmini_frame) - 2))
			< sizeof(struct tfmini_frame) - 2) {
		printf("Error, could not read tmf, rc = %d\n", rc);
		printf("Value gotten tfm->distl=%d\n", tmf->dist_l);
	}

	datas = tmf;

	for (i = 0; i < 8; i ++) {
		checksum += datas[i];
	}

	if ((checksum & 0xFF) != tmf->checksum) {
		synchronize(fd, tmf);
	}

        return 0;
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
int ucs_distance_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];
    int fd;                        // OLIMEX_TFMINI file descriptor
	// struct sched_param param;
    struct tfmini_frame tmf;
	unsigned int value;

    // Create watchdog
    WDOG_ID wdog = wd_create ();
    if( wdog != NULL) {
        printf(" Watchdog created \n");
        wd_start( wdog, WATCHDOG_TIME_SEC * TICK_PER_SEC, &wdog_handler, 0);
    }

    // Opening the TFMini device
	fd = open(OLIMEX_TFMINI, O_RDONLY);
	printf("Starting distance publisher application\n");
	if (fd < 0) {
		printf("Could not open %s\n", OLIMEX_TFMINI);
        return 0;
	}
     if (synchronize(fd, &tmf)) {
		printf("Could not synchronize %s\n", OLIMEX_TFMINI);
        return 0;
    }
    if(strlen(DISTANCE_AGENT_INET6_ADDR) > 39 || strlen(DISTANCE_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(DISTANCE_NODE) > 39 || strlen(DISTANCE_TOPIC) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, DISTANCE_AGENT_UDP_PORT);
    strcpy(inet6_address, DISTANCE_AGENT_INET6_ADDR);
    strcpy(node_name, DISTANCE_NODE);
    strcpy(topic_name, DISTANCE_TOPIC);

#if (!defined(CONFIG_FS_ROMFS) || !defined(CONFIG_NSH_ROMFSETC))
    printf("device ID - %d, nOde - %s, topic - %s \n", DISTANCE_DEVICE_ID, node_name, topic_name );
    // Initialize 6lowpan when running on nsh prompt
    init_distance_6lowpan();
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

    printf("micro-ROS Publisher\r\n");

    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name, &publisher_ops));

    printf(" Distance pub_main \n");
    std_msgs__msg__Int32 msg;
    do {
    	while (synchronize(fd, &tmf)) {
			printf("Error during synchro \n");
    	}
		value = (tmf.dist_h << 8 | tmf.dist_l);
		if (value < 12000) {
			msg.data = value;
		}
        rv = rcl_publish(&publisher, (const void*)&msg, NULL);
        if (RCL_RET_OK == rv )
        {
            printf("TFMINI sent: '%i'\n", msg.data);
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

    printf("\r\nClosing Micro-ROS 6lowpan app\r\n");
    soft_reset();
    return 0;
}
