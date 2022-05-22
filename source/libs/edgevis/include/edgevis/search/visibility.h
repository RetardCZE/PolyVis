#include "edgevis/search/expansion.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"

#include "polyanya/parsers/map_parser.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

namespace edgevis{
    class EdgeVisibility{
    public:
        EdgeVisibility(Mesh& mesh);
        ~EdgeVisibility();


        std::vector<Point> find_visibility(int edge_id, bool side);
        void set_visual_mesh(const parsers::GeomMesh &gmesh);
        bool switch_debug(bool on);
        void precompute_edges();
        const Mesh& mesh_reference();
    private:
        void expand(SearchNode &node, std::vector<Point> &visibility, int level);
        void visualise(parsers::GeomMesh &mesh, SearchNode start, SearchNode expanded);

        Mesh mesh;
        parsers::GeomMesh gmesh;
        bool debug;
    };

};