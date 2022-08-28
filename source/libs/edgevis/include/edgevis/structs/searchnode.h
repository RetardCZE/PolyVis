#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/search/robust_geometry.h"
#include "edgevis/search/intersections.h"
#include "edgevis/structs/intersection.h"
#include "edgevis/structs/searchpoint.h"

namespace edgevis
{
    struct SearchNode
    {

        SearchPoint rootR, rootL;  // root
        SearchNode* predecessor;
        SearchPoint transitionL, transitionR; // transitionL transitionR based on rootR rootL orientation
        // child becomes transition when expanding searchnode

        // The transitionL and transitionR vertex of the edge the interval is lying on.
        // When generating the successors of this node, end there.
        int leftVertex ,rightVertex;

        int leftRootVertex, rightRootVertex;

        // Index of the polygon we're going to "push" into.
        // Every successor must lie within this polygon.
        int nextPolygon;
        int comingFrom;

        /*
        friend std::ostream& operator<<(std::ostream& stream, const SearchNode& sn)
        {
            return stream << "SearchNode - root: " << sn.rootL << " | " << sn.rootR << "\n"
                          << "edge: " << sn.transitionL << " | " << sn.transitionR << "\n"
                          << "next: " << sn.nextPolygon << "\n";
        }*/
    };

    typedef SearchNode* SearchNodePtr;

}
