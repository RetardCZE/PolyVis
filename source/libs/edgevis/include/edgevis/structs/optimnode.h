#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/searchnode.h"

namespace edgevis
{
    struct OptimNodeV1
    {
        SearchPoint root_R, root_L;  // root
        SearchPoint P;

        bool operator==(OptimNodeV1& A) const {
            if (P == A.P && root_R == A.root_R && root_L == A.root_L) {
                return true;
            } else {
                return false;
            }
        }
        bool operator!=(OptimNodeV1& A) const {
            if (P == A.P && root_R == A.root_R && root_L == A.root_L) {
                return false;
            } else {
                return true;
            }
        }

        /*
        friend std::ostream& operator<<(std::ostream& stream, const OptimNode& on)
        {
            return stream << "OptimNode - point: " << on.P << "\n"
                          << "roots: " << on.rootL << " | " << on.rootR << "\n"
                          << "pivot: " << on.pivot_L << " | " << on.pivot_R <<"\n";
        }
         */
    };

    struct OptimNodeV2
    {
        SearchPoint root_R, root_L;  // root
        SearchPoint P;
        bool isAlwaysVisible;
        /*
        friend std::ostream& operator<<(std::ostream& stream, const OptimNode& on)
        {
            return stream << "OptimNode - point: " << on.P << "\n"
                          << "roots: " << on.rootL << " | " << on.rootR << "\n"
                          << "pivot: " << on.pivot_L << " | " << on.pivot_R <<"\n";
        }
         */
    };

    typedef OptimNodeV1* OptimNodeV1Ptr;
    typedef OptimNodeV2* OptimNodeV2Ptr;

}
