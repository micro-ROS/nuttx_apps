#include <nuttx/config.h>

#include <stdio.h>
#include <time.h>

#include "pong_server.h"

#if defined(BUILD_MODULE)
extern "C" int main(int argc, char *argv[])
#else
extern "C" int uros_pong_server_main(int argc, char* argv[])
#endif
{
    try {
        PongServer ps(argc, argv);        
        ROS_INFO("%s\n", "Pong server initialized.");
              
        while(true) {
            ps.spin(1000);
        }

    } catch(kobuki::RCLException& ex) {
        fprintf(stderr, "%s\n", ex.what());
        return -1;
    }

    return 0;
}
