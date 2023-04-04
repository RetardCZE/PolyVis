#pragma once
#include "edgevis/structs/point.h"

namespace edgevis{
    struct SegmentLineIntersection{
        /*
         * Assuming we are using search mesh we can define all needed intersections with mesh vertices.
         * We assume oriented line a -> b. In case of segment we assume parent -> child or right -> left orientation,
         * but it should not matter.
         *
         * It is crucial to keep a -> b oriented in direction of forward expansion.
         */

        int a, b, c, d; // -1, -2 if free points

        bool operator==(SegmentLineIntersection& A) const {
            if (a == A.a && b == A.b && c == A.c && d == A.d){
                return true;
            } else {
                return false;
            }
        }

    };
}
