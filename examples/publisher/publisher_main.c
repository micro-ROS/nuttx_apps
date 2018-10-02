#include <nuttx/config.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int publisher_main(int argc, char* argv[])
#endif
{
    (void)argc;
    (void)argv;
    const char* a = "";
    rclc_init(1, a);
    rclc_node_t* node     = rclc_create_node("publisher_node", "");
    //rclc_publisher_t* publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "publisher_example", 1);
    const rosidl_message_type_support_t* c = rosidl_typesupport_micrortps_c__get_message_type_support_handle__std_msgs__msg__Int32();
    //const rosidl_message_type_support_t* b = rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32();
    const rclc_message_type_support_t type_support = RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_publisher_t* publisher = rclc_create_publisher(node, type_support, "publisher_example", 1);

    std_msgs__msg__Int32 msg;
    std_msgs__msg__Int32__init(&msg);

    while (rclc_ok())
    {
        printf("Sending: '%i'\n", msg.data++);       
        rclc_publish(publisher, (const void*)&msg);
        rclc_spin_node_once(node, 500);
    }
    rclc_destroy_publisher(publisher);
    rclc_destroy_node(node);
    return 0;
}
