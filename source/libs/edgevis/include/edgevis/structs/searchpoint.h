#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/intersection.h"

namespace edgevis {
    struct SearchPoint {
        int p;
        SegmentLineIntersection i;
        bool isIntersection;
    };
}