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
    std::vector<SearchNode> right_nodes, left_nodes;
    std::vector<OptimNodeV1> rightOptimNodesV1, leftOptimNodesV1;
    std::vector<OptimNodeV2> rightOptimNodesV2, leftOptimNodesV2;
    std::vector<OptimNodeV3> rightOptimNodesV3, leftOptimNodesV3;
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
