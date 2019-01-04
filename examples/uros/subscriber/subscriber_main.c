#include <nuttx/config.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

static int count = 0;

void on_message(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    printf("I heard: [%i]\n", msg->data);
    count++;
}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int subscriber_main(int argc, char* argv[])
#endif
{
    (void)argc;
    (void)argv;
    int result = 0;
    rclc_init(1, "");
    rclc_node_t* node = NULL;
    if (node = rclc_create_node("int32_subscriber_c", ""))
    {
        rclc_subscription_t* sub = NULL;
        if(sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                                        "std_msgs_msg_Int32", on_message, 1, false))
        {
            while (rclc_ok() && count <= 50)
            {
                rclc_spin_node_once(node, 500);
            }
            rclc_destroy_subscription(sub);
        }
        else
        {
            printf("Issues creating subscriber\n");
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
