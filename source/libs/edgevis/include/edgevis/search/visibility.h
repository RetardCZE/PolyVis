#include "edgevis/search/expansion.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/optimnode.h"
#include "edgevis/helpers/geometry.h"

#include "polyanya/parsers/map_parser.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

namespace cgm = cairo_geom_drawer;
namespace edgevis{
    class EdgeVisibility{
        enum struct STATE
        {
            RIGHT,
            VISIBLE,
            LEFT,
        };
    public:
        EdgeVisibility(Mesh& mesh);
        ~EdgeVisibility();


        std::vector<OptimNode> find_visibility(int edge_id, bool side);
        void set_visual_mesh(const parsers::GeomMesh &gmesh);
        bool switch_debug(bool on);
        void precompute_edges();
        const Mesh& mesh_reference();
        std::vector<Point> find_point_visibility(Point p, std::vector<Point> visu, bool debug);

        void visualise_segment(Point A, Point B, int color, float opacity);
        void visualise_point(Point A, int color);
        void visualise_polygon(std::vector<Point>& p, int color);
        void reset_visu();
    private:
        void expand(SearchNode &node, std::vector<OptimNode> &visibility, int level, bool side);

        std::vector<Edge*> get_init_edges(PointLocation pl);
        Edge current_edge;

        int save_cntr = 0;

        Mesh mesh;
        Point last_point;
        cgm::CairoGeomDrawer cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 20);
        parsers::GeomMesh gmesh;
        bool debug;
    };
    void recompute_roots(SearchNode &node, EdgeVisibility *eObject);
    int expand_searchnode(SearchNode &node, const Mesh &mesh, SearchNode *newNodes, edgevis::EdgeVisibility *eObject);
};