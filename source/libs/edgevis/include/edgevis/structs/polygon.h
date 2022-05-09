#pragma once
#include <vector>

namespace edgevis
{

struct Polygon
{
    // "int" here means an array index.
    std::vector<int> vertices;
    std::vector<int> polygons;
    std::vector<int> edges;

    bool is_one_way;
    double min_x, max_x, min_y, max_y;
};

}
