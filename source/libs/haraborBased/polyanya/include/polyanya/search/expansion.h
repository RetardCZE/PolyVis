#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/point.h"
#include <vector>

namespace polyanya
{

int expand(SearchNode& node, const Point& start, const Mesh& mesh,
           Successor* successors);
}
