#ifndef __THIN_KOBUKI_NODE_H__
#define __THIN_KOBUKI_NODE_H__

#include <rcl/rcl.h>

namespace kobuki {
    // keeping as a struct for easier "C" interfacing
    struct KobukiNode {
        KobukiNode(int argc, char* argv[]);
        ~KobukiNode();

        rcl_context_t context;
        rcl_node_t node;
    };
}

#endif  // __THIN_KOBUKI_NODE_H__