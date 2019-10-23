#include <nuttx/config.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#define CHECK_RET(FUNC) ret = FUNC ; if(ret != RMW_RET_OK) { fprintf(stderr, "Error invoking FUNC %d\n", ret); return -1; }

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int publisher_main(int argc, char* argv[])
#endif
{
    rmw_ret_t ret = RMW_RET_OK;
    int result = 0;
    printf("Initializing...");
    CHECK_RET(rclc_init(argc, argv));
    printf("done\n");
    const rclc_message_type_support_t type_support = RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_node_t* node = NULL;

    printf("Creating node...");
    if (!(node = rclc_create_node("publisher_node", "")))
    {
        printf("failed\n");
        result = -1;
        goto exit;
    }
    printf("done\n");

    rclc_publisher_t* publisher = NULL;
    printf("Creating publisher...");
    if(!(publisher = rclc_create_publisher(node, type_support, "std_msgs_msg_Int32", 1)))
    {
        printf("failed\n");
        result = -1;
        goto exit;
    }
    printf("done\n");

    std_msgs__msg__Int32 msg;
    msg.data = 0;
    while (true ) // rclc_ok() && msg.data <= 1000)
    {
        printf("Sending: '%i'\n", msg.data++);
        rclc_publish(publisher, (const void*)&msg);
        rclc_spin_node_once(node, 500);
        if (msg.data == 1000) { 
            msg.data = 0; 
        }
    }
    CHECK_RET(rclc_destroy_publisher(publisher));

exit:
    CHECK_RET(rclc_destroy_node(node));

    return result;
}
