#pragma once
#include "edgevis/search/expansion.h"
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
#include "geom/colors.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"
#include <opencv2/opencv.hpp>

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

        void visualiseOnline(Point p);

        void
        find_arbitrary_edge_visibility(int edgeParent, int edgeChild, std::vector<SearchNode> &leftVis,
                                       std::vector<SearchNode> &rightVis);
        std::vector<SearchNode> find_edge_visibility(int edge, bool side);

        void precompute_edges_searchnodes();
        const Mesh& mesh_reference();
        void precompute_edges_optimnodesV1();
        void precompute_edges_optimnodesV2();
        std::vector<Point> find_point_visibility_optim1(Point p, bool debug, double &steps, int &debugEdge);
        std::vector<Point> find_point_visibility_optim2(Point p, bool debug, double &steps);
        Point evaluate_intersection(SearchPoint& sp);

        void set_visual_mesh(const parsers::GeomMesh &gmesh);
        void visualise_segment(Point A, Point B, int color, float opacity);
        void visualise_point(Point A, int color, bool draw);
        void visualise_heat_point(Point A, double heat, double heightIncrement, double widthIncrement);
        void visualise_named_point(Point A, int color, std::string str);
        void visualise_vertex_indexes();
        void visualise_heatmap(std::vector<Point> &points, std::vector<double> &compT, double tMax, double tMin,
                               std::string name);
        void visualise_polygon(std::vector<Point> &p, int color, bool draw);
        void reset_visu();
        int widthCV = 1200;

    private:
        std::vector<OptimNodeV1> compute_side_optimnodesV1(Edge &edge, bool right=true);
        void expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side);
        void get_arbitrary_edge_init_nodes(Edge edge, int &rightCount, int &leftCount,
                                           edgevis::SearchNode *initLeftNodes,
                                           edgevis::SearchNode *initRightNodes);
        int get_edge_init_nodes(Edge edge, bool side, SearchNode *initNodes);
        int find_visible(SearchNode &node, std::vector<int> &sorted_vertices, int *right_visible, int *left_visible);
        int expand_forward(SearchNode &node, SearchNode *newNodes);
        void back_propagation(SearchNode &node);
        void compute_optimnodesv1(SearchNode &node, OptimNodeV1 &o1, OptimNodeV1 &o2);
        bool check_visibility_on2(Point &a, OptimNodeV2 &on, Edge &e, bool right);

        std::vector<Edge*> get_init_edges(PointLocation pl);
        Edge current_edge;
        std::vector<SearchNode*> deleteQueue;
        bool searchnodes_precomputed = false;
        bool optimnodes1_precomputed = false;
        bool optimnodes2_precomputed = false;
        int save_cntr = 0;

        int windowWidth = 1400;
        int windowHeight = 900;

        Mesh mesh;
        Point last_point;
        cgm::CairoGeomDrawer cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 20);
        parsers::GeomMesh gmesh;
        bool debug;
    };
};