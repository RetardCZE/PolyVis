#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/intersection.h"

namespace edgevis
{

struct SearchPoint{
    int p;
    robust_geom::SegmentLineIntersection i;
    bool is_intersection = false;
};

struct SearchNode
{

    SearchPoint root_R, root_L;  // root
    SearchNode* predecessor;
    SearchPoint child_L, child_R; // child_L child_R based on root_R root_L orientation
    // child becomes transition when expanding searchnode

    // The child_L vertex of the edge the interval is lying on.
    // When generating the successors of this node, end there.
    int left_vertex;

    // The child_R vertex of the edge the interval is lying on.
    // When generating the successors of this node, start there.
    int right_vertex;

    // Index of the polygon we're going to "push" into.
    // Every successor must lie within this polygon.
    int next_polygon;
    int coming_from;

    /*
    friend std::ostream& operator<<(std::ostream& stream, const SearchNode& sn)
    {
        return stream << "SearchNode - root: " << sn.root_L << " | " << sn.root_R << "\n"
                      << "edge: " << sn.child_L << " | " << sn.child_R << "\n"
                      << "next: " << sn.next_polygon << "\n";
    }*/
};

typedef SearchNode* SearchNodePtr;

}
