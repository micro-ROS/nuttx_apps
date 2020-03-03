#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uROS libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rmw_uros/options.h>

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int uros_6lowpan_main(int argc, char* argv[])
#endif
{
    char buffer[256]; // Buffer to save configuration commands.
    char aux_buffer[2]; // Buffer to save the configuration ID.

    if(3 > argc || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port sub/pub [<max_topics>]\n");
        return 0;
    }
    //Check if the IP and the port are correct
    if(strlen(argv[1])>39 || strlen(argv[2])>4){
        printf("Error: IP or port size incorrect \r\n");
        return 0;
    }

    //6lowpan configuration process
    printf("Do you want to configure the 6lowpan network? (Y/N)\r\n");
    memset(buffer,0,sizeof(buffer));
    scanf("%s", buffer);

    if(!strcmp(buffer,"y")){
        system("ifdown wpan0"); // Is necessary to bring down the network to configure.
        system("i8sak wpan0 startpan cd:ab"); //Set the radio as an endpoint.
        system("i8sak set chan 26"); //Set the radio channel.
        system("i8sak set panid cd:ab"); //Set network PAN ID.

        // Set the ID
        printf("Introduce your 6LowPan ID (It must between 00 and FF (Hex))\r\n");
        scanf("%s", aux_buffer);
        if (strlen(aux_buffer) < 2) {
            printf("Error ID must be between 00 and FF\n");
            return 0;
        }
        sprintf(buffer,"i8sak set saddr 42:%c%c",aux_buffer[0],aux_buffer[1]); // Set the short address of the radio
        system(buffer);
        sprintf(buffer, "i8sak set eaddr 00:fa:de:00:de:ad:be:%c%c", aux_buffer[0],aux_buffer[1]); // TODO: This won't work on the lastest version of NuttX
        system(buffer);
        system("i8sak acceptassoc");
        system("ifup wpan0"); // Bring up the network.
        system("mount -t procfs /proc");// Mount the proc file system to check the connection data.

        printf("Connection data\r\n");
        system("cat proc/net/wpan0");
        //6lowpan configuration finished
    }

    //Waiting for a user input to continue.
    printf("Press any key to continue\r\n");
    scanf("%s",aux_buffer);

    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();

    rv = rcl_init_options_init(&options, rcl_get_default_allocator());
    if (RCL_RET_OK != rv) {
        printf("rcl init options error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    // Set the IP and the port of the Agent
    // TODO (Juan) Check if the IP and the port are correct
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options);

    rcl_context_t context = rcl_get_zero_initialized_context();
    rv = rcl_init(0, NULL, &options, &context);
    if (RCL_RET_OK != rv) {
        printf("rcl initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    if(!strcmp(argv[3],"pub")){
        printf("micro-ROS Publisher\r\n");

        rcl_node_options_t node_ops = rcl_node_get_default_options();
        rcl_node_t node = rcl_get_zero_initialized_node();
        rv = rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops);
        if (RCL_RET_OK != rv) {
            printf("Node initialization error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
        rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
        rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &publisher_ops);
        if (RCL_RET_OK != rv) {
            printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        std_msgs__msg__Int32 msg;
        const int num_msg = 1000;
        msg.data = 0;
        usleep(3000000); // As we are sending low number mensajes we need to wait discovery of the subscriber. (Do not have a notification on discovery)
        do {
            rv = rcl_publish(&publisher, (const void*)&msg, NULL);
            if (RCL_RET_OK == rv )
            {
                printf("Sent: '%i'\n", msg.data++);
                sleep(1);
            }
        } while (RCL_RET_OK == rv && msg.data < num_msg );
        printf("TOTAL sent: %i\n", num_msg);

        rv = rcl_publisher_fini(&publisher, &node);
        rv = rcl_node_fini(&node);

    }
    else if(!strcmp(argv[3],"sub")){
        printf("micro-ROS subscriber \r\n");

        rcl_node_options_t node_ops = rcl_node_get_default_options();
        rcl_node_t node = rcl_get_zero_initialized_node();
        rv = rcl_node_init(&node, "int32_subscriber_rcl", "", &context, &node_ops);
        if (RCL_RET_OK != rv)
        {
            fprintf(stderr, "[main] error in rcl : %s\n", rcutils_get_error_string().str);
            rcl_reset_error();
            return 1;
        }

        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
        rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
        rv = rcl_subscription_init(
            &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &subscription_ops);
        if (RCL_RET_OK != rv) {
            printf("Subscription initialization error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        rv = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
        if (RCL_RET_OK != rv) {
            printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        rv = rcl_wait_set_clear(&wait_set);
        if (RCL_RET_OK != rv) {
            printf("Wait set clear error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        size_t index;
        rv = rcl_wait_set_add_subscription(&wait_set, &subscription, &index);
        if (RCL_RET_OK != rv) {
            printf("Wait set add subscription error: %s\n", rcl_get_error_string().str);
            return 1;
        }

        void* msg = rcl_get_default_allocator().zero_allocate(sizeof(std_msgs__msg__Int32), 1, rcl_get_default_allocator().state);
        do {
 
            rv = rcl_wait(&wait_set, 1000000);
            if(RCL_RET_SUBSCRIPTION_TAKE_FAILED == rv){
                printf("foo\r\n");
            }
            else{
                printf("%i\r\n",rv);
            }
            for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
                rv = rcl_take(wait_set.subscriptions[i], msg, NULL, NULL);
                if (RCL_RET_OK == rv)
                {
                    printf("I received: [%i]\n", ((const std_msgs__msg__Int32*)msg)->data);
                }
            }
        } while ( RCL_RET_OK == rv || RCL_RET_SUBSCRIPTION_TAKE_FAILED == rv);

        rv = rcl_subscription_fini(&subscription, &node);
        rv = rcl_node_fini(&node);   
    }
    else{
        printf("Error. It must be pub (publisher) or sub (subscriber).\r\n");
    }

    printf("Closing Micro-ROS 6lowpan app\r\n");
    return 0;
}