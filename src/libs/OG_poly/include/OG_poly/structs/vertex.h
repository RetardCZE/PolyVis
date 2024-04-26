#pragma once
#include "OG_poly/structs/point.h"
#include <vector>

namespace OG_poly
{

// A point in the polygon mesh.
struct Vertex
{
    Point p;
    // "int" here means an array index.
    std::vector<int> polygons;

    bool is_corner;
    bool is_ambig;
};

}
