/**
 * File:    utils.cpp
 *
 * Date:   25.03.2021
 * Author:  Lukas Fanta
 * E-mail:  fantalukas2108@gmail.com
 *
 */

#include "edgevis/parsers/utils.h"

int parsers::UnionFind::find(int x) {
    if (x == -1) {
        return -1;
    }
    if (parent[x] != x) {
        parent[x] = find(parent[x]);
    }
    return parent[x];
}

// can't use "union" as that's a keyword!
// also: don't use union by rank as we need find(x) == x after merge.
void parsers::UnionFind::merge(int x, int y) {
    x = find(x);
    y = find(y);
    parent[y] = x;
}

void parsers::computeLimits(const parsers::GeomMesh &map, double &xMin, double &yMin, double &xMax, double &yMax) {
    xMin = std::numeric_limits<double>::max();
    yMin = std::numeric_limits<double>::max();
    xMax = std::numeric_limits<double>::lowest();
    yMax = std::numeric_limits<double>::lowest();
    for (const auto &v : map.vertices) {
        if (v.point.x < xMin) xMin = v.point.x;
        if (v.point.y < yMin) yMin = v.point.y;
        if (xMax < v.point.x) xMax = v.point.x;
        if (yMax < v.point.y) yMax = v.point.y;
    }
}