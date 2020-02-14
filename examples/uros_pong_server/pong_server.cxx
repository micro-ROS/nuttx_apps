#include "pong_server.h"

#include "rosidl_generator_c/string_functions.h"
#include <sstream>

using namespace kobuki;

static rcl_publisher_t * pub_ptr;
void pong_callback(const void * msgin)
{
  rcl_ret_t rc;
  const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
  if (msg == NULL) {
    ROS_INFO("pong_server: %s\n", "received NULL message.");
  } else {
    // ROS_INFO("pong_server: received pong message.\n");
    rcl_publish(pub_ptr, msg, NULL);
    if (rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(pong_callback);
    }
  }   
}

PongServer::PongServer(int argc, char* argv[]) : 
  node(rcl_get_zero_initialized_node()),
  publisher(rcl_get_zero_initialized_publisher()),
  subscriber(rcl_get_zero_initialized_subscription()),
  executor(rclc_executor_get_zero_initialized_executor())
{
    // RCL NODE INITIALIZATION
    rcl_ret_t rc = RMW_RET_OK;

    ROS_INFO("pong_server: %s\n", "Initializing rclc");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rc = rclc_support_init(&support, argc, argv, &allocator);
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        throw RCLException("Failed to create executor support object");
    }

    ROS_INFO("pong_server: %s\n", "Creating node");
    rc = rclc_node_init_default(&node, "pong_server", "", &support);
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to create node");
    }
    // COMMUNICATION INIT
    ROS_INFO("pong_server: %s\n", "Creating publisher");
    const rosidl_message_type_support_t *pub_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    
    rc = rclc_publisher_init_default(
        &publisher,
        &node,
        pub_ts,
        "uros_pong");

    if (rc != RCL_RET_OK) {        
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to create pong publisher");
    }

    // save publisher in global pointer to publish msg in callback
    pub_ptr = &publisher;
    
    ROS_INFO("pong_server: %s\n", "Creating subscriber");
    const rosidl_message_type_support_t *sub_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    
    rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        sub_ts,
        "uros_ping");
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to create ping subscriber");
    }

    ROS_INFO("pong_server: %s\n", "Initializing message");
    if(!std_msgs__msg__Header__init(&sub_msg)) {
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to initialize message");
    }
    const size_t BUFSIZE = 1024;
    void * data = realloc(sub_msg.frame_id.data, BUFSIZE);
    if(data != NULL) {
        sub_msg.frame_id.data = (char*)data;
        sub_msg.frame_id.capacity = BUFSIZE;
    } else {
        std_msgs__msg__Header__fini(&sub_msg);
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to reallocate message buffer");
    }

    ROS_INFO("pong_server: %s\n", "Creating executor");
    unsigned int num_handles = 1;
    rc = rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        std_msgs__msg__Header__fini(&sub_msg);
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to create executor");
    }

    rc = rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, pong_callback, ON_NEW_DATA);
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        std_msgs__msg__Header__fini(&sub_msg);
        WARN_RET(rclc_executor_fini( &executor))
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        WARN_RET(rclc_support_fini(&support))
        throw RCLException("Failed to add subscription to executor");
    }
}

PongServer::~PongServer()
{
    std_msgs__msg__Header__fini(&sub_msg);
    WARN_RET(rcl_subscription_fini( &subscriber, &node))
    WARN_RET(rcl_publisher_fini( &publisher, &node))
    WARN_RET(rcl_node_fini( &node))
    WARN_RET(rclc_executor_fini( &executor))
    WARN_RET(rclc_support_fini(&support))
}

bool PongServer::wait(uint32_t timeout_ms)
{
    return rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timeout_ms));
}