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
int subscriber_main(int argc, char* argv[])
#endif
{
    rcl_ret_t rv = RCL_RET_ERROR;

#ifdef CONFIG_UROS_TRANSPORT_UDP
    struct in_addr addr;
    netlib_ifdown("eth0");
    addr.s_addr = HTONL(0xc0a80a03);
    netlib_set_ipv4addr("eth0", &addr);
    netlib_ifup("eth0");
    system("ping -c2 192.168.10.1");
#endif //
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()));

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context));

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "int32_subscriber_rcl", "", &context, &node_ops));

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &subscription_ops));

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()));
  sleep(2);

  std_msgs__msg__Int32 msg;
  msg.data = 0;
  do {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t index;
    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index));
    
    rcl_wait(&wait_set, 1000);
     usleep(10000);
    if (wait_set.subscriptions[index]) {
      rcl_ret_t rc = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
      if (RCL_RET_OK == rc) {
        printf("I received: [%i]\n", msg.data);
      }
    }
  } while ( msg.data != 100 );

  RCCHECK(rcl_subscription_fini(&subscription, &node));
  RCCHECK(rcl_node_fini(&node));

  sys_trace_ctf_meas_stop();

  return 0;
}
