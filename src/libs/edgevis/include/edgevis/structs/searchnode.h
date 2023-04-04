#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/helpers/robust_geometry.h"
#include "edgevis/helpers/intersections.h"
#include "edgevis/structs/intersection.h"
#include "edgevis/structs/searchpoint.h"

namespace edgevis
{
    struct SearchNode
    {
        SearchPoint rootR, rootL;  // root
        SearchNode* predecessor;
        SearchPoint transitionL, transitionR;

        int leftVertex ,rightVertex;
        int leftRootVertex, rightRootVertex;

        int nextPolygon;
        int comingFrom;

        friend std::ostream& operator<<(std::ostream& stream, const SearchNode& sn)
        {
            return stream << "SearchNode - root: " << sn.rootL << " | " << sn.rootR << "\n"
                          << "edge: " << sn.transitionL << " | " << sn.transitionR << "\n"
                          << "next: " << sn.nextPolygon << "\n";

        }
    };

    typedef SearchNode* SearchNodePtr;

}
