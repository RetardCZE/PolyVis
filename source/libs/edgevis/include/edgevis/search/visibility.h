#include "edgevis/search/expansion.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"

#include "polyanya/parsers/map_parser.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

namespace edgevis{
    std::vector<Point> find_visibility(int edge_id, const Mesh &mesh, bool side, parsers::GeomMesh &gmesh, bool debig);
    void expand(SearchNode &node, const Mesh &mesh, std::vector<Point> &visibility, int level, parsers::GeomMesh &gmesh,
                bool debug);
    void visualise(parsers::GeomMesh &mesh, SearchNode start, SearchNode expanded);
};