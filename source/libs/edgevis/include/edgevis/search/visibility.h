#pragma once
#include "edgevis/search/visibility_utils.h"
#include "edgevis/search/intersections.h"

#include "edgevis/structs/edge.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/optimnode.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/intersection.h"

#include "edgevis/helpers/geometry.h"

#include <vector>

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


        std::vector<SearchNode> find_edge_visibility(int edge_id, bool side);
        void set_visual_mesh(const parsers::GeomMesh &gmesh);
        bool switch_debug(bool on);
        void precompute_edges_searchnodes();
        const Mesh& mesh_reference();
        /*
        std::vector<Point> find_point_visibility(Point p, std::vector<Point> visu, bool debug);
        */
        void visualise_segment(Point A, Point B, int color, float opacity);
        void visualise_point(Point A, int color);
        void visualise_polygon(std::vector<Point>& p, int color);
        void reset_visu();
    private:
        void expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side);
        int get_edge_init_nodes(Edge edge, bool side, SearchNode *initNodes);
        int find_visible(SearchNode &node, std::vector<int> &sorted_vertices, int *right_visible, int *left_visible);
        int expand_forward(SearchNode &node, SearchNode *newNodes);
        void back_propagation(SearchNode &node);

        std::vector<Edge*> get_init_edges(PointLocation pl);
        Edge current_edge;

        int save_cntr = 0;

        Mesh mesh;
        Point last_point;
        cgm::CairoGeomDrawer cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 20);
        parsers::GeomMesh gmesh;
        bool debug;
    };
};