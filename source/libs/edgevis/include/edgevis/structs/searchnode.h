#pragma once
#include "edgevis/structs/point.h"

namespace edgevis
{

// A search node.
// Only makes sense given a mesh and an endpoint, which the node does not store.
// This means that the f value needs to be set manually.
struct SearchNode
{

    Point parent, child;  // root
    Point left, right; // left right based on parent child orientation

    // The left vertex of the edge the interval is lying on.
    // When generating the successors of this node, end there.
    int left_vertex;

    // The right vertex of the edge the interval is lying on.
    // When generating the successors of this node, start there.
    int right_vertex;

    // Index of the polygon we're going to "push" into.
    // Every successor must lie within this polygon.
    int next_polygon;


    friend std::ostream& operator<<(std::ostream& stream, const SearchNode& sn)
    {
        return stream << "SearchNode - root: " << sn.parent << " | " << sn.child << "\n"
                                     <<"edge: " << sn.left << " | " << sn.right << "\n"
                                     <<"next: " << sn.next_polygon << "\n";
    }
};

typedef SearchNode* SearchNodePtr;

}
