#pragma once
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/vertex.h"
#include "edgevis/search/robust_geometry.h"
#include "edgevis/search/intersections.h"

using namespace  edgevis;
namespace robust_geom{
    struct SegmentLineIntersection{
        /*
         * Assuming we are using edgevis mesh we can define all needed intersections with mesh vertices.
         * We assume oriented line a -> b. In case of segment we assume parent -> child or right -> left orientation,
         * but it should not matter.
         */
        int a, b, c, d;
        bool is_calculated = false;

        bool calculate(Mesh &mesh);
        Point p;
    };

}