#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/edge.h"
#include "edgevis/helpers/geometry.h"
#include <vector>

namespace edgevis
{
bool is_on_abscissa(Point A, Point B, Point C);

/*
 * side means on which side of edge we want to look since we want to calculate visibility separately.
 * lets say true means right and false means left
 */
int get_edge_init_nodes(Edge edge, bool side, const Mesh& mesh, SearchNode* initNodes);

/*
 *
 */
bool is_observable(Point left_parent,
                       Point left_child,
                       Point right_parent,
                       Point right_child,
                       Point p);

/*
 *
 */
std::vector<int> normalise_ids(std::vector<int> elements, int new_start, int& offset);

/*
 *
 */
void recompute_roots(SearchNode* nodes, SearchNode parent, int num);

/*
 *
 */
int expand_searchnode(SearchNode node, const Mesh& mesh, SearchNode* newNodes);

}
