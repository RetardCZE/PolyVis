#pragma once

#include "edgevis/structs/polygon.h"
#include "edgevis/structs/vertex.h"
#include "edgevis/structs/edge.h"
#include "edgevis/helpers/robust_geometry.h"
#include "edgevis/helpers/edgevis_utils.h"

#include "geomMesh/parsers/utils.h"

#include "draw/colors.h"
#include "draw/cairo_geom_drawer.h"
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <map>
#include <memory>
#include <random>

namespace cgm = cairo_geom_drawer;
namespace edgevis {
    struct TEAExpansion{
        std::vector<SearchNode> SearchNodes;
    };

    struct PEAExpansion {
        std::vector<SearchNode> SearchNodes;
    };

    struct PolyContainment {
        enum Type {
            // Does not use any ints.
            OUTSIDE,

            // Does not use any ints.
            INSIDE,

            // Uses adjacent_poly, vertex1 and vertex2.
            ON_EDGE,

            // Uses vertex1.
            ON_VERTEX,
        };

        Type type;

        int adjacent_poly;

        // If on edge, vertex1/vertex2 represents the left/right vertices of the
        // edge when looking from a point in the poly.
        int vertex1, vertex2;

        friend std::ostream &operator<<(std::ostream &stream,
                                        const PolyContainment &pc) {
            switch (pc.type) {
                case PolyContainment::OUTSIDE:
                    return stream << "OUTSIDE";

                case PolyContainment::INSIDE:
                    return stream << "INSIDE";

                case PolyContainment::ON_EDGE:
                    return stream << "ON_EDGE (poly " << pc.adjacent_poly
                                  << ", vertices " << pc.vertex1 << ", "
                                  << pc.vertex2 << ")";

                case PolyContainment::ON_VERTEX:
                    return stream << "ON_VERTEX (" << pc.vertex1 << ")";

                default:
                    assert(false);
                    return stream;
            }
        }
    };

    struct PointLocation {
        enum Type {
            // Does not use any ints.
            NOT_ON_MESH,

            // Uses poly1 (the polygon it is on).
            IN_POLYGON,

            // Uses poly1 (the polygon it is on) and both vertices.
            ON_MESH_BORDER,       // edge: a polygon is not traversable

            // Uses poly1, poly2 and both vertices.
            ON_EDGE,              // edge: both polygons are traversable

            // Uses vertex1.
            // Can use poly1 to specify the "grid corrected poly".
            // Will need to manually assign poly1, though.
            ON_CORNER_VERTEX_AMBIG,   // vertex; two+ polygons are not traversable

            // Uses vertex1. Also returns an arbirary traversable adjacent
            // polygon in poly1.
            ON_CORNER_VERTEX_UNAMBIG, // vertex; one polygon is not traversable

            // Uses vertex1. Also returns an arbitrary adjacent polygon in poly1.
            ON_NON_CORNER_VERTEX, // vertex: all polygons are traversable
        };

        Type type;
        int poly1, poly2;
        // If on edge, vertex1/vertex2 represents the left/right vertices of the
        // edge when looking from a point in poly1.
        int vertex1, vertex2;

        friend std::ostream &operator<<(std::ostream &stream,
                                        const PointLocation &pl) {
            switch (pl.type) {
                case PointLocation::NOT_ON_MESH:
                    return stream << "NOT_ON_MESH";

                case PointLocation::IN_POLYGON:
                    return stream << "IN_POLYGON (" << pl.poly1 << ")";

                case PointLocation::ON_MESH_BORDER:
                    return stream << "ON_MESH_BORDER (poly " << pl.poly1
                                  << ", vertices " << pl.vertex1 << ", "
                                  << pl.vertex2 << ")";

                case PointLocation::ON_EDGE:
                    return stream << "ON_EDGE (polys "
                                  << pl.poly1 << ", " << pl.poly2 << ", vertices "
                                  << pl.vertex1 << ", " << pl.vertex2 << ")";

                case PointLocation::ON_CORNER_VERTEX_AMBIG:
                    return stream << "ON_CORNER_VERTEX_AMBIG (" << pl.vertex1
                                  << ", poly? " << pl.poly1 << ")";

                case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
                    return stream << "ON_CORNER_VERTEX_UNAMBIG (" << pl.vertex1
                                  << ", poly " << pl.poly1 << ")";

                case PointLocation::ON_NON_CORNER_VERTEX:
                    return stream << "ON_NON_CORNER_VERTEX (" << pl.vertex1
                                  << ", poly " << pl.poly1 << ")";

                default:
                    assert(false);
                    return stream;
            }
        }

        bool operator==(const PointLocation &other) const {
            if (type != other.type) {
                return false;
            }

            switch (type) {
                case PointLocation::NOT_ON_MESH:
                    return true;

                case PointLocation::IN_POLYGON:
                    return poly1 == other.poly1;

                case PointLocation::ON_MESH_BORDER:
                    if (poly1 != other.poly1) {
                        return false;
                    }
                    if (vertex1 == other.vertex1 && vertex2 == other.vertex2) {
                        return true;
                    }
                    if (vertex1 == other.vertex2 && vertex2 == other.vertex1) {
                        return true;
                    }
                    return false;

                case PointLocation::ON_EDGE:
                    if (poly1 == other.poly1 && poly2 == other.poly2 &&
                        vertex1 == other.vertex1 && vertex2 == other.vertex2) {
                        return true;
                    }
                    if (poly1 == other.poly2 && poly2 == other.poly1 &&
                        vertex1 == other.vertex2 && vertex2 == other.vertex1) {
                        return true;
                    }
                    return false;

                case PointLocation::ON_CORNER_VERTEX_AMBIG:
                case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
                case PointLocation::ON_NON_CORNER_VERTEX:
                    return vertex1 == other.vertex1;

                default:
                    assert(false);
                    return false;
            }
        }

        bool operator!=(const PointLocation &other) const {
            return !((*this) == other);
        }
    };
    class Mesh {
        enum struct STATE
        {
            RIGHT,
            VISIBLE,
            LEFT,
        };
    private:
        std::map<double, std::vector<int>> slabs;
        double min_x, max_x, min_y, max_y;
        int* sortedV;
        int* sortedP;
        Point r, l ,b;
        int visSize = 0;
        SearchNode temp;
        std::vector<Point> vis;

        // 1 for PEA and TEA, 2 for edges (only arbitrary ones)
        std::vector<Point> free_points;

        bool searchnodes_precomputed = false;
        bool optimnodes1_precomputed = false;
        bool optimnodes2_precomputed = false;
        bool optimnodes3_precomputed = false;

        int leveller;
        bool debug = false;
        Edge current_edge;
        std::vector<SearchNode*> deleteQueue;

        /********************************************
         * mesh.cpp
         */
        void read(const parsers::GeomMesh &geomMesh);
        void calculate_edges();
        void precalc_point_location();
        /********************************************/

        /********************************************
         * edgevis_preprocessing.cpp
         */
        std::vector<OptimNode> compute_side_optimnodesV1(Edge &edge, bool right=true);
        void compute_optimnodesv1(SearchNode &node, OptimNode &o1, OptimNode &o2);
        bool check_visibility_on2(Point &a, OptimNode &on, Edge &e, bool right);
        void compute_side_optimnodesV3(Edge &edge, bool right=true);
        /********************************************/

        /********************************************
         * edge_visibility.cpp
         */
        void get_arbitrary_edge_init_nodes(Edge edge, int &rightCount, int &leftCount,
                                           edgevis::SearchNode *initLeftNodes,
                                           edgevis::SearchNode *initRightNodes);
        int get_edge_init_nodes(Edge edge, bool side, SearchNode *initNodes);
        int get_point_init_nodes(Point root, SearchNode *initNodes);
        void expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side, int level, bool draw);

        int expand_forward(SearchNode &node, SearchNode *newNodes);
        void back_propagation(SearchNode &node);
        /********************************************/

        /********************************************
         * point_visibility.cpp
         */
        std::vector<Edge*> get_init_edges(PointLocation pl);
        void expand_TEA(SearchNode &n, int level);
        void expand_PEA(SearchNode &n, int level);
        int expand_TEA_once(SearchNode &node, std::vector<SearchNode> &newNodes);
        int expand_PEA_once(SearchNode& node, std::vector<SearchNode> &newNodes);
        /********************************************/

        /********************************************
         * edgevis_utils.cpp
         */
        int find_visible(SearchNode &node, int *right_visible, int *left_visible, int vCount);
        int find_visible_binary_tree(SearchNode &node, const int offset, int *right_visible, int *left_visible,
                                     bool *right_collinear, bool *left_collinear);
        void init_temp_node(SearchNode &node);
        /********************************************/
    public:
        int T, P;
        std::vector<Vertex> mesh_vertices;
        std::vector<Polygon> mesh_polygons;
        std::vector<Edge> mesh_edges;
        int max_poly_sides;
        bool useRobustOrientatation = true;

        std::vector<TEAExpansion> allocTEA;
        int TEAItems = 100;
        std::vector<PEAExpansion> allocPEA;
        int PEAItems = 100;
        void realloc_TEA_mem(int items);
        void realloc_PEA_mem(int items);

        /********************************************
         * mesh.cpp
         */
        Mesh(const parsers::GeomMesh &geomMesh);
        ~Mesh();
        bool is_convex();
        Point random_point(std::mt19937 &seed);
        PolyContainment poly_contains_point(int poly, Point &p);
        PointLocation get_point_location(Point &p);
        PointLocation get_point_location_naive(Point &p);
        Point evaluate_intersection(SearchPoint& sp);
        /********************************************/

        /********************************************
         * edgevis_preprocessing.cpp
         */
        void precompute_edges_searchnodes();
        void precompute_edges_optimnodesV1();
        void precompute_edges_optimnodesV2();
        void precompute_edges_optimnodesV3();
        /********************************************/

        /********************************************
         * edge_visibility.cpp
         */
        std::vector<SearchNode> find_edge_visibility(int edge, bool side, bool draw);
        void find_arbitrary_edge_visibility(int edgeParent, int edgeChild, std::vector<SearchNode> &leftVis,
                                            std::vector<SearchNode> &rightVis);
        /********************************************/

        /********************************************
         * point_visibility.cpp
         */
        std::vector<Point> find_point_visibility_optim1(Point p, bool debug, double &steps, int &debugEdge);
        std::vector<Point> find_point_visibility_optim2(Point p, bool debug, double &steps, int &debugEdge);
        std::vector<Point> find_point_visibility_optim3(Point p, bool debug, double &steps, int &debugEdge);
        std::vector<Point> find_point_visibility_optim4(Point p);
        std::vector<Point> find_point_visibility_TEA(Point p, bool debug);
        std::vector<Point> find_point_visibility_PEA(Point p, bool debug);
        /********************************************/

        void set_visual_mesh(const parsers::GeomMesh &gmesh);
        void visualise_segment(Point A, Point B, int color, float opacity);
        void visualise_point(Point A, int color, bool draw);
        void visualise_polygon(std::vector<Point> &p, int color, bool draw);
        void reset_visu();

        cgm::CairoGeomDrawer cgm_drawer = cgm::CairoGeomDrawer(this->max_x, this->max_y, 20);
        parsers::GeomMesh gmesh;

    };
}
