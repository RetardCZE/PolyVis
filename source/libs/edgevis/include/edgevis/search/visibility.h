#include "edgevis/search/expansion.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"

namespace edgevis{
    std::vector<Point> find_visibility( int edge_id, const Mesh& mesh, bool side);
    void expand(SearchNode node, const Mesh& mesh, std::vector<Point>& visibility, int level);
};