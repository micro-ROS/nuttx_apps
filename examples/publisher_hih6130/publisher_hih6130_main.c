#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/sensors/hih6130.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

int fd_temp; // HIH6130 file descriptor
struct hih6130_s sample; // Data structure of the sensor

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int publisher_hih6130_main(int argc, char *argv[])
#endif
{
  (void)argc;
  (void)argv;
  int result = 0;
  rclc_init(1, "");
  const rclc_message_type_support_t type_support =
      RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  rclc_node_t *node = NULL;
  //Creating publisher node
    fd_temp=open("/dev/hih6130", O_RDONLY);
  if(fd_temp<0){
    printf("Error opening HIH6130 sensor %d\n",fd_temp);
  }
  if (node = rclc_create_node("publisher_node", "")) {
    rclc_publisher_t *publisher = NULL;
    if (publisher = rclc_create_publisher(node, type_support,
                                          "std_msgs_msg_Int32", 1)) {
      read(fd_temp, &sample, sizeof(uint32_t));
      std_msgs__msg__Int32 msg;
      msg.data = sample.temp;

      while (1) {
        //Read temperature sampl
        read(fd_temp, &sample, sizeof(uint32_t));
        //Save in the data variable
        msg.data = sample.temp;
        printf("Temperature: '%i'\n", msg.data);
        rclc_publish(publisher, (const void *)&msg);
        sleep(1);
        rclc_spin_node_once(node, 500);
      }
      rclc_destroy_publisher(publisher);
    } else {
      printf("Issues creating publisher\n");
      result = -1;
    }
    rclc_destroy_node(node);
  } else {
    printf("Issues creating node\n");
    result = -1;
  }
  return result;
}
