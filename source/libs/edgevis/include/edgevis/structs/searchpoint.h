#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/intersection.h"

namespace edgevis {
    struct SearchPoint {
        int p;
        SegmentLineIntersection i;
        bool isIntersection;

        bool operator==(SearchPoint& A) const {
            if (!isIntersection && !A.isIntersection && p == A.p) {
                return true;
            } else if (isIntersection && A.isIntersection && i == A.i) {
                return true;
            } else {
                return false;
            }
        }
    };
}