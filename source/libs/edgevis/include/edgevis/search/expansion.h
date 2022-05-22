#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/edge.h"
#include "edgevis/helpers/geometry.h"
#include <vector>

namespace edgevis
{
bool is_on_segment(Point A, Point B, Point C);

/*
 * side means on which side of edge we want to look since we want to calculate visibility separately.
 * let say true mean child_R and false means child_L
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
int normalise(const Polygon& P, int transition_R, std::vector<int>* sorted_vertices, std::vector<int>* sorted_polygons);

int find_visible(const Mesh& mesh, SearchNode& node, std::vector<int>& sorted_vertices, int* right_visible, int* left_visible);

/*
 *
 */
void recompute_roots(SearchNode &node);


SearchNode init_temp_node(SearchNode& node);
/*
 *
 */
int expand_searchnode(SearchNode& node, const Mesh& mesh, SearchNode* newNodes);

}
