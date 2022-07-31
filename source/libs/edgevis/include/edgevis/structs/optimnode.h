#pragma once
#include "edgevis/structs/point.h"

namespace edgevis
{
    struct OptimNode
    {
        Point root_R, root_L;  // root
        Point P;
        Point pivot_R, pivot_L;
    };

    typedef OptimNode* OptimNodePtr;

}
