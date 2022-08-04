#pragma once
#include "edgevis/structs/point.h"

namespace edgevis
{
    struct OptimNode
    {
        Point root_R, root_L;  // root
        Point P;
        Point pivot_R, pivot_L;

        friend std::ostream& operator<<(std::ostream& stream, const OptimNode& on)
        {
            return stream << "OptimNode - point: " << on.P << "\n"
                          << "roots: " << on.root_L << " | " << on.root_R << "\n"
                          << "pivot: " << on.pivot_L << " | " << on.pivot_R <<"\n";
        }

    };

    typedef OptimNode* OptimNodePtr;

}
