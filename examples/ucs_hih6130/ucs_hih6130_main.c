#include <nuttx/config.h>
#include <nuttx/sensors/hih6130.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
// #include <demo_msgs/msg/demo_hih6130.h>
#include <std_msgs/msg/int32.h>

#include "init_hih_6lowpan.h"
#include "hih6130.h"

#define LED_HEARTBEAT					(0x00)

static void led_toggle(void) {
	static int status = 0;
	static int half_seconds = 0;

	if (half_seconds == 1) {
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
int ucs_hih6130_main(int argc, char* argv[])
#endif
{
    char udp_port[5];
    char inet6_address[40];
    char node_name[40];
    char topic_name[40];
    char topic_name2[40];
    int fd_temp;                //HIH6130 file descriptor
    struct hih6130_s sample;    //Data structure of the sensor

    // Opening the hih6130sensor with read only permission
    fd_temp=open("/dev/hih6130", O_RDONLY);
    if(fd_temp<0){
        printf("Error opening HIH6130 sensor %d\n",fd_temp);
        return 0;
    }    

    if(strlen(HIH_AGENT_INET6_ADDR) > 39 || strlen(HIH_AGENT_UDP_PORT) > 4) {
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }
    if(strlen(HIH_NODE) > 39 || strlen(HIH_TOPIC) > 39 || strlen(HIH_TOPIC2) > 39) {
        printf("Error: Node or topic size incorrect \r\n");
        return 0;
    }

    // Initialize 6lowpan
    init_hih_6lowpan();

    // Define agent's udp port and IPv6 address, then uros node and topic names. 
    strcpy(udp_port, HIH_AGENT_UDP_PORT);
    strcpy(inet6_address, HIH_AGENT_INET6_ADDR);
    strcpy(node_name, HIH_NODE);
    strcpy(topic_name, HIH_TOPIC);
    strcpy(topic_name2, HIH_TOPIC2);
    printf("device ID - %d, nOde - %s, topic - %s \n", HIH_DEVICE_ID, node_name, topic_name );

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
    // rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(demo_msgs, msg, DemoHih6130), topic_name, &publisher_ops);
    rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name, &publisher_ops);
    if (RCL_RET_OK != rv) {
        printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }
    rcl_publisher_t publisher2 = rcl_get_zero_initialized_publisher();
    // rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(demo_msgs, msg, DemoHih6130), topic_name, &publisher_ops);
    rv = rcl_publisher_init(&publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name2, &publisher_ops);
    if (RCL_RET_OK != rv) {
        printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }


    printf(" HIH6130 pub_main \n");
	// demo_msgs__msg__DemoHih6130 msg;
	// msg.heartbeats = 0;
    std_msgs__msg__Int32 msg;
    do {
		int num = 10;

        //Read sensor sampl
        read(fd_temp, &sample, sizeof(uint32_t));
        // msg.temp = sample.temp;                
        // msg.hum = sample.hum;  
        // msg.heartbeats++;              
		msg.data = sample.temp;
        rv = rcl_publish(&publisher, (const void*)&msg, NULL);
        if (RCL_RET_OK == rv )
        {
            // printf("Sent: Temperature %d, Humidity %d  \n", msg.temp, msg.hum);
            printf("Sent: Temperature %d \n", msg.data);
            if (RCL_RET_OK == rv )
    	    {
		        msg.data = sample.hum;
                rv = rcl_publish(&publisher2, (const void*)&msg, NULL);
                if (RCL_RET_OK == rv )
                {
                    printf("Sent: Humidity %d  \n", msg.data);
                    led_toggle();
    		        // num millisec
	    	        usleep(1000 * num);
                }
            }
        }
    } while (RCL_RET_OK == rv);

    rv = rcl_publisher_fini(&publisher, &node);
    rv = rcl_node_fini(&node);


    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}