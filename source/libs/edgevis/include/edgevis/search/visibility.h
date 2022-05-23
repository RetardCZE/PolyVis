#include "edgevis/search/expansion.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"

#include "polyanya/parsers/map_parser.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"
namespace cgm = cairo_geom_drawer;
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
        std::vector<Point> find_point_visibility(Point p, std::vector<Point> visu);
    private:
        void expand(SearchNode &node, std::vector<Point> &visibility, int level);
        void visualise_segment(Point A, Point B, int color, float opacity);
        void visualise_point(Point A, int color);
        void visualise_polygon(std::vector<Point>& p, int color);
        void reset_visu();
        std::vector<Edge*> get_init_edges(Point p);

        int save_cntr = 0;
        Mesh mesh;
        cgm::CairoGeomDrawer cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 20);
        parsers::GeomMesh gmesh;
        bool debug;
    };

};