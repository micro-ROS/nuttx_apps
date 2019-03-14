#include <nuttx/config.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#include <geometry_msgs/msg/quaternion.h>
#include <sensor_msgs/msg/imu.h>

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int myapp_main(int argc, char* argv[])
#endif
{
    (void)argc;
    (void)argv;
    int result = 0;
    rclc_init(1, "");
    printf("Turtlebot publisher\n");
    //const rclc_message_type_support_t type_support = RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_node_t* node = NULL;
    if (node = rclc_create_node("myapp_node", ""))
    {
        rclc_publisher_t* publisher       = NULL;
        rclc_publisher_t * pub_q          = NULL; 
        rclc_publisher_t * pub_imu        = NULL;        

	bool quarternion_ok = false; 
        bool imu_ok         = false;

        if (pub_q = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "quaternion", 1)){
          quarternion_ok = true;
          printf("Created Quarternion publisher \n");
        } else {
          printf("Failed to create Quarternion publisher \n");
        }        


        if (pub_imu = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu", 1) )
        {
          imu_ok = true;
          printf("Created IMU publisher \n");
        } else {
          printf("Failed to create IMU publisher.\n");
        }  
    
        if ( quarternion_ok && imu_ok )
        {
         while (true) {
           geometry_msgs__msg__Quaternion msg_quaternion;
           geometry_msgs__msg__Quaternion__init(&msg_quaternion);
           msg_quaternion.x = 7.5;
           rclc_publish(pub_q, (const void *)&msg_quaternion);
	   printf("Sending Quarternion and IMU  \n" );
           sensor_msgs__msg__Imu msg_imu;
           sensor_msgs__msg__Imu__init(&msg_imu);
           rclc_publish(pub_imu, (const void *)&msg_imu);
           rclc_spin_node_once(node, 500);
        }
       }
       rclc_destroy_publisher(pub_q);
       rclc_destroy_publisher(pub_imu);
       rclc_destroy_node(node);
       return 0;

   }
 return 0;
}
