#include <nuttx/config.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/header.h>
#include <stdio.h>
#include <time.h>

struct timespec send_time;

void msg_cb(const std_msgs__msg__Header* msg)
{
    struct timespec receive_time;
    clock_gettime(CLOCK_MONOTONIC, &receive_time);

    printf("Current time: %d, %d\n", receive_time.tv_sec, receive_time.tv_nsec);
    printf("Message time: %d, %d\n", msg->stamp.sec, msg->stamp.nanosec);

    float msg_time = (float)(msg->stamp.sec) + RCL_NS_TO_S((float)msg->stamp.nanosec);
    float  rec_time = (float)(receive_time.tv_sec) + RCL_NS_TO_S((float)receive_time.tv_nsec);

    printf("Delay: %f\n", (rec_time - msg_time));
}

#define CHECK_RET(FUNC) ret = FUNC ; if(ret != RMW_RET_OK) { fprintf(stderr, "Error invoking FUNC %d\n", ret); return -1; }

#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
    fprintf(stderr, "[" #rclc "] error in " #rcl ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int delay_test_main(int argc, char* argv[])
#endif
{
    rcl_ret_t ret = RMW_RET_OK;
    int result = 0;
    
    printf("Initializing %s...", argv[0]);
    CHECK_RET(rclc_init(argc, argv));
    printf("done\n");
    
    const rclc_message_type_support_t type_support = RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    rclc_node_t* node = NULL;

    printf("Creating node...");
    if (!(node = rclc_create_node("uros_delay_test", "")))
    {
        printf("failed\n");
        result = -1;
        goto exit;
    }
    printf("done\n");

    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    
    ret = rcl_subscription_init(
        &subscriber,
        &node->rcl_node,
        sub_type_support,
        "uros_delay_test",
        &subscription_ops);
    
    if(ret != RCL_RET_OK) {
        printf("failed\n");
        result = -1;
        goto exit;
    }

    rclc_publisher_t* publisher = NULL;
    printf("Creating publisher...");
    if(!(publisher = rclc_create_publisher(node, type_support, "uros_delay_test", 1)))
    {
        printf("failed\n");
        result = -1;
        goto exit;
    }
    printf("done\n");

    
    sleep(2);

    result = clock_gettime(CLOCK_MONOTONIC, &send_time);
    std_msgs__msg__Header msg = { .stamp.sec = send_time.tv_sec, .stamp.nanosec = send_time.tv_nsec };

    rclc_publish(publisher, &msg);

    // get empty wait set
    uint32_t timeout_ms = 1000;

    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
        PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_init);
        result = -1;
        goto exit;
    }

    // set rmw fields to NULL
    ret = rcl_wait_set_clear(&wait_set);
    if (ret != RCL_RET_OK) {
        PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_clear_subscriptions);
        result = -1;
        goto exit;
    }

    size_t index = 0; // is never used - denotes the index of the subscription in the storage container
    ret = rcl_wait_set_add_subscription(&wait_set, &subscriber, &index);
    if (ret != RCL_RET_OK) {
        PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_add_subscription);
        result = -1;
        goto exit;
    }

    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
    if (ret == RCL_RET_TIMEOUT) {
        result = -1;
        goto exit;
    }					

    if (ret != RCL_RET_OK) {
        PRINT_RCL_ERROR(spin_node_once, rcl_wait);
        result = -1;
        goto exit;
    }

    // if RET_OK => one subscription is received because only ONE subscription has been added
    // just double check here.
    if ( wait_set.subscriptions[0] ){
        std_msgs__msg__Header sub_msg;
        rmw_message_info_t        messageInfo;
        ret = rcl_take(&subscriber, &sub_msg, &messageInfo);

        if (ret != RCL_RET_OK) {
            PRINT_RCL_ERROR(spin_node_once, rcl_take);
            result = -1;
            goto exit;
        }

        // call message callback
        msg_cb( &sub_msg );
        
    } else {
        //sanity check
        fprintf(stderr, "[spin_node_once] no subscription received.\n");
    }
    rcl_wait_set_fini(&wait_set);
            

    printf("Done, terminating\n");
    rclc_destroy_publisher(publisher);

exit:
    rclc_destroy_node(node);

    return result;
}
