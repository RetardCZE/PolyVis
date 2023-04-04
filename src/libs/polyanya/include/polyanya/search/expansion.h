#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/point.h"
#include "polyanya/search/robust_geometry.h"
#include "polyanya/helpers/geometry.h"
#include "polyanya/structs/vertex.h"
#include "polyanya/structs/consts.h"
#include <vector>

namespace polyanya
{
    Orientation get_orientation_switchable(const Point& a, const Point& b, const Point& c, bool robust);

    int expand(SearchNode &node, const Point &start, const Mesh &mesh, Successor *successors, bool robust);
}
