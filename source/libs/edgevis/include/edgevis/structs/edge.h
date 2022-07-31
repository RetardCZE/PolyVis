#pragma once
#include "edgevis/structs/vertex.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/optimnode.h"
#include <vector>

using namespace edgevis;
struct Edge
{
    int parent, child;
    int rightPoly, leftPoly;
    // std::vector<Point> right_visibility, left_visibility;
    std::vector<OptimNode> right_nodes, left_nodes;
    bool operator==(Edge& A) const {
        if (parent == A.parent && child == A.child) {
            return true;
        } else if (parent == A.child && child == A.parent) {
            return true;
        } else {
            return false;
        }
    }
};
