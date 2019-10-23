#include "ros_util.h"
#include "thin_kobuki_node.h"

namespace kobuki
{
    KobukiNode::KobukiNode(int argc, char* argv[]) 
    : context(rcl_get_zero_initialized_context()), node(rcl_get_zero_initialized_node()) {
        rcl_init_options_t init_options;    //global static var in rcl
        rcl_ret_t          rc;
        init_options = rcl_get_zero_initialized_init_options();
        rc = rcl_init_options_init(&init_options, rcl_get_default_allocator());
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }

        rc = rcl_init(argc, argv, &init_options, &context);
        rcl_init_options_fini(&init_options);
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }

        rcl_node_options_t node_ops = rcl_node_get_default_options();

<<<<<<< HEAD
                if((count % 5) == 0) {
                    rcl_publish( &pub_odom, (const void *) &pose, NULL);
                }

                // imu data
                //float heading;
                //robot.getImu(heading, vtheta);
                //imu.header.seq = seq; //header.seq does not exist
                //imu.header.stamp = timestamp;
                //imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading);
                //imu.angular_velocity.z = vtheta;

                //rclc_publish( pub_imu, (const void *) &imu); 
                //printf("Sending imu\n");
                // debug - sending a twist message - because odom and imu data produce error messages in nsh shell as well as in ros2-environment
                //rclc_publish ( pub_twist, (const void *) & msg_twist);
                //printf("Sending kobuki_twist\n");
                }

            //spin once
            //rclc_spin_node_once(node, 10);
            // use do{}while(0) to have port error handling 
            // by replacing 'return' in error detection parts rclc_spin_node_once  by a 'break')
            
            do {
                uint32_t timeout_ms = 10;

                // get empty wait set
                rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
                rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_init);
                    break;
                }

                // set rmw fields to NULL
                rc = rcl_wait_set_clear(&wait_set);
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_clear_subscriptions);
                    _spin_node_exit(&wait_set);
                    break;
                }

                size_t index = 0; // is never used - denotes the index of the subscription in the storage container
                rc = rcl_wait_set_add_subscription(&wait_set, &sub_cmd_vel, &index);
                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_wait_set_add_subscription);
                    _spin_node_exit(&wait_set);
                    break;
                }

                rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
                if (rc == RCL_RET_TIMEOUT) {
                    _spin_node_exit(&wait_set);
                    break;
                }					

                if (rc != RCL_RET_OK) {
                PRINT_RCL_ERROR(spin_node_once, rcl_wait);
                _spin_node_exit(&wait_set);
                break;
            }

            // if RET_OK => one subscription is received because only ONE subscription has been added
            // just double check here.
            if ( wait_set.subscriptions[0] ){
                geometry_msgs__msg__Twist msg;
                rmw_message_info_t        messageInfo;
                rc = rcl_take(&sub_cmd_vel, &msg, &messageInfo, NULL);

                if (rc != RCL_RET_OK) {
                    PRINT_RCL_ERROR(spin_node_once, rcl_take);
                    _spin_node_exit(&wait_set);
                    break;
                }

                // call message callback
                commandVelCallback( &msg );
                
            } else {
                //sanity check
                fprintf(stderr, "[spin_node_once] no subscription received.\n");
            }
            rcl_wait_set_fini(&wait_set);
            
            } while (0);
        }        
=======
        rc = rcl_node_init(&node, "uros_kobuki_node", "", &context, &node_ops);
        if (rc != RCL_RET_OK) {
            throw RCLException(rcutils_get_error_string().str);
        }
>>>>>>> Cleaning up Kobuki driver
    }

    KobukiNode::~KobukiNode() {
        rcl_node_fini( &node);
    }
}