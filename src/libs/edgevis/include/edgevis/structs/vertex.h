#pragma once
#include "edgevis/structs/point.h"
#include <vector>

using namespace edgevis;
// A point in the polygon mesh.
struct Vertex
{
    Point p;
    // "int" here means an array index.
    std::vector<int> polygons;
    std::vector<int> edges;

    bool is_corner;
    bool is_ambig;
};

