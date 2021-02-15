#include <nuttx/config.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rmw_uros/options.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/tracing/tracing_probes.h>

#ifdef CONFIG_UROS_TRANSPORT_UDP
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>

#include "netutils/netlib.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int publisher_main(int argc, char* argv[])
#endif
{
    rcl_ret_t rv;
#ifdef BM_PWR
    sys_trace_ctf_meas_pwr();
#endif // BM_PWR

#ifdef CONFIG_UROS_TRANSPORT_UDP
    struct in_addr addr;
    netlib_ifdown("eth0");
    addr.s_addr = HTONL(0xc0a80a02);
    netlib_set_ipv4addr("eth0", &addr);
    netlib_ifup("eth0");
    sleep(4);
// wait to get the udpated 
    system("ping -c1 192.168.10.1");
#endif //
    sys_trace_ctf_meas_start();
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    rv = rcl_init_options_init(&options, rcl_get_default_allocator());
        printf("rcl_init_options_init\n");

    if (RCL_RET_OK != rv) {
        printf("rcl init options error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_context_t context = rcl_get_zero_initialized_context();
    rv = rcl_init(argc, argv, &options, &context);
    printf("rcl_init\n");

    if (RCL_RET_OK != rv) {
        printf("rcl initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_node_options_t node_ops = rcl_node_get_default_options();

    

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops));

    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
//    publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &publisher_ops));

    std_msgs__msg__Int32 msg;
    const int num_msg = 50;
    msg.data = 0;


    sleep(2); // Sleep a while to ensure DDS matching before sending request
    rcl_ret_t rc;
    do {
	    rc = rcl_publish(&publisher, (const void*)&msg, NULL);
	    if (RCL_RET_OK == rc ) {
		    printf("Sent: '%i'\n", msg.data++);
	    }
#ifdef BM_PWR
	    sys_trace_ctf_meas_pwr();
#endif //BM_PWR
    } while (RCL_RET_OK == rc && msg.data < num_msg );
    printf("TOTAL sent: %i\n", num_msg);

    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    sys_trace_ctf_meas_stop();

    return 0;
}
