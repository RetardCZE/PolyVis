#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/searchnode.h"

namespace edgevis
{
    struct OptimNode
    {
        SearchPoint root_R, root_L;  // root
        SearchPoint P;
        bool isAlwaysVisible;
        float root_R_order, root_L_order;
        bool TESTER = false;
        bool OT1 = false;
        bool OT2 = false;

        bool operator==(OptimNode& A) const {
            if (P == A.P && root_R == A.root_R && root_L == A.root_L) {
                return true;
            } else {
                return false;
            }
        }
        bool operator!=(OptimNode& A) const {
            if (P == A.P && root_R == A.root_R && root_L == A.root_L) {
                return false;
            } else {
                return true;
            }
        }

    };


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

    struct OptimNodeV3
    {
        SearchPoint root_R, root_L;  // root
        SearchPoint P;
        float root_R_order, root_L_order;
        bool isAlwaysVisible;

        bool operator==(OptimNodeV2& A) const {
            if (P == A.P && root_R == A.root_R && root_L == A.root_L) {
                return true;
            } else {
                return false;
            }
        }
        bool operator!=(OptimNodeV2& A) const {
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

    typedef OptimNodeV1* OptimNodeV1Ptr;
    typedef OptimNodeV2* OptimNodeV2Ptr;
    typedef OptimNodeV3* OptimNodeV3Ptr;

}
