#pragma once

#include "edgevis/structs/mesh.h"
#include <vector>

namespace edgevis
{
    int normalise(const Polygon& P, int transition_R, int *sorted_vertices, int *sorted_polygons);

    SearchNode init_temp_node(SearchNode& node);

}
