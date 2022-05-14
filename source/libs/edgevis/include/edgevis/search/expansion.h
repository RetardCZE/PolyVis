#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/edge.h"
#include <vector>

namespace edgevis
{
/*
 * side means on which side of edge we want to look since we want to calculate visibility separately.
 * lets say true means right and false means left
 */
int get_edge_init_nodes(Edge edge, bool side, const Mesh& mesh, SearchNode* initNodes);

/*
 *
 */
std::vector<int> normalise_ids(std::vector<int> elements, int new_start);
}
