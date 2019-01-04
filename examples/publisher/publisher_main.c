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
    int result = 0;
    rclc_init(1, "");
    const rclc_message_type_support_t type_support = RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_node_t* node = NULL;
    if (node = rclc_create_node("publisher_node", ""))
    {
        rclc_publisher_t* publisher = NULL;
        if(publisher = rclc_create_publisher(node, type_support, "std_msgs_msg_Int32", 1))
        {

            std_msgs__msg__Int32 msg;
            msg.data = 0;
            while (rclc_ok() && msg.data <= 1000)
            {
                printf("Sending: '%i'\n", msg.data++);
                rclc_publish(publisher, (const void*)&msg);
                rclc_spin_node_once(node, 500);
            }
            rclc_destroy_publisher(publisher);
        }
        else
        {
            printf("Issues creating publisher\n");
            result = -1;
        }
        rclc_destroy_node(node);
    }
    else
    {
        printf("Issues creating node\n");
        result = -1;
    }
    return result;
}
