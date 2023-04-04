#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/intersection.h"

namespace edgevis {
    struct SearchPoint {
        int p = 0;
        SegmentLineIntersection i;
        bool isIntersection;


        friend std::ostream& operator<<(std::ostream& stream, const SearchPoint& sp)
        {
            if(sp.isIntersection){
                return stream << "SearchPoint - intersection: " << sp.i.a << ", " << sp.i.b << ", " << sp.i.c << ", " << sp.i.d <<"\n";
            }else{
                return stream << "SearchPoint - point: " << sp.p << "\n";
            }
        }

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