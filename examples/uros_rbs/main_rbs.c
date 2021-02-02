// Copyright (c) 2020 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pthread.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>


#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){\
   printf("Failed status on line %d: %d. Message: %s, Aborting.\n",__LINE__,(int)temp_rc, rcl_get_error_string().str);\
  rcutils_reset_error(); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCUNUSED(fn) { rcl_ret_t temp_rc __attribute__((unused)); temp_rc = fn; }


rcl_subscription_t high_ping_subscription_;
std_msgs__msg__Int32 high_ping_msg_;
rcl_publisher_t high_pong_publisher_;
rcl_subscription_t low_ping_subscription_;
std_msgs__msg__Int32 low_ping_msg_;
rcl_publisher_t low_pong_publisher_;


void burn_cpu_cycles(long duration)
{
  if (duration > 0) {
    clockid_t clockId;
    clockId = CLOCK_REALTIME;
    // clock_getcpuclockid(pthread_self(), &clockId);
    // NuttX does not implement this function
    // -lrt  is not available , I did not find this function in 
    // NuttX subfolders
    // adding EXTRA_LIBS to the Makefile did not resolve the error.
    struct timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != rand() && x % 10 != 0) {
        x++;
      }
      struct timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      long currentDuration = (currentTimeP.tv_sec - startTimeP.tv_sec) * 1000000000 + (currentTimeP.tv_nsec - startTimeP.tv_nsec);
      doAgain = (currentDuration < duration);
    }
  }
}


void high_ping_received(const void * pong_msg)
{
  // burn_cpu_cycles(100000000);  // TODO: Get this value from parameter 'high_busyloop'.
  printf("high ping received.\n");
  RCUNUSED(rcl_publish(&high_pong_publisher_, pong_msg, NULL));
}


void low_ping_received(const void * pong_msg)
{
  // burn_cpu_cycles(100000000);  // TODO: Get this value from parameter 'low_busyloop'.
  printf("low ping received.\n");
  RCUNUSED(rcl_publish(&low_pong_publisher_, pong_msg, NULL));
}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int uros_rbs_main(int argc, char* argv[])
#endif
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  printf("Welcome to RBS Demo!\n");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "pong_rclc_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&high_pong_publisher_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "high_pong"));
  RCCHECK(rclc_subscription_init_best_effort(&high_ping_subscription_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "high_ping"));

  RCCHECK(rclc_publisher_init_best_effort(&low_pong_publisher_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "low_pong"));
  RCCHECK(rclc_subscription_init_best_effort(&low_ping_subscription_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "low_ping"));

  rclc_executor_t high_executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&high_executor, &support.context, 2, &allocator));
  
  rclc_executor_sched_param_t sparam_high, sparam_low;
  sparam_high.priority = sched_get_priority_max(SCHED_FIFO);
  sparam_low.priority = sched_get_priority_min(SCHED_FIFO);
  printf("max prio %d min prio %d\n", SCHED_PRIORITY_MAX, SCHED_PRIORITY_MIN);
  printf("high prio %d low prio %d\n", sparam_high.priority, sparam_low.priority);

  sparam_high.priority = 20;
  sparam_low.priority = 10;
  printf("high prio %d low prio %d\n", sparam_high.priority, sparam_low.priority);

  printf("subscription high ping \n");
  RCCHECK(rclc_executor_add_subscription_sched(&high_executor, &high_ping_subscription_, &high_ping_msg_, &high_ping_received, ON_NEW_DATA, &sparam_high));
   printf("subscription low ping \n");
  RCCHECK(rclc_executor_add_subscription_sched(&high_executor, &low_ping_subscription_, &low_ping_msg_, &low_ping_received, ON_NEW_DATA, &sparam_low));
    printf("start executor \n");
  RCCHECK(rclc_executor_start_multi_threading_for_nuttx(&high_executor));
  printf("clean up \n");
  RCCHECK(rclc_executor_fini(&high_executor));
  RCCHECK(rcl_subscription_fini(&high_ping_subscription_, &node));
  RCCHECK(rcl_publisher_fini(&high_pong_publisher_, &node));
  RCCHECK(rcl_subscription_fini(&low_ping_subscription_, &node));  
  RCCHECK(rcl_publisher_fini(&low_pong_publisher_, &node));

  RCCHECK(rcl_node_fini(&node));
}
/*
Error message missing library rt for clock_getcpuclockid(pthread_self(), &clockId);
but this function is not implemented in NuttX! (01-02-2021)

arm-none-eabi-ld --entry=__start -nostartfiles -nodefaultlibs -g -T/home/jst3si/olimex_ws/firmware/NuttX/configs/olimex-stm32-e407/scripts/ld.script -L"/home/jst3si/olimex_ws/firmware/NuttX/staging" -L"/home/jst3si/olimex_ws/firmware/NuttX/arch/arm/src/board" -L "/usr/lib/gcc/arm-none-eabi/6.3.1/../../../arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard" \
	-o "/home/jst3si/olimex_ws/firmware/NuttX/nuttx"   \
	--start-group -lsched -ldrivers -lconfigs -lc -lmm -larch -lxx -lapps -lnet -lfs -lbinfmt -lxx -lboard -lsupc++ "/usr/lib/gcc/arm-none-eabi/6.3.1/thumb/v7e-m/fpv4-sp/hard/libgcc.a" --end-group
/home/jst3si/olimex_ws/firmware/NuttX/staging/libapps.a(main_rbs.o): In function `burn_cpu_cycles':
/home/jst3si/olimex_ws/firmware/apps/examples/uros_rbs/main_rbs.c:49: undefined reference to `clock_getcpuclockid'
Makefile:184: recipe for target 'nuttx' failed
make[1]: *** [nuttx] Error 1
make[1]: Verzeichnis „/home/jst3si/olimex_ws/firmware/NuttX/arch/arm/src“ wird verlassen
tools/Makefile.unix:422: recipe for target 'pass2' failed
make: *** [pass2] Error 2


*/
