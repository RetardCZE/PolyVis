#include "edgevis/search/visibility.h"

namespace edgevis{
    void
    expand(SearchNode &node, const Mesh &mesh, std::vector<Point> &visibility, int level, parsers::GeomMesh &gmesh,
           bool debug) {
        if(debug)
        std::cout << node  << "\n";

        if(node.next_polygon == -1){
            visibility.push_back(node.right);
            visibility.push_back(node.left);
            if(debug)
            std::cout << node.left << " | " << node.right << std::endl;
            return;
        }
        int num;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::expand_searchnode(node, mesh, nodes);

        for(int i = 0; i < num; i++){
            if(debug) {std::cout << nodes[i] << " -- " <<std::endl;
                visualise(gmesh, node, nodes[i]);
                getchar();
            }
            recompute_roots(nodes[i], num);
            if(debug) {std::cout << nodes[i] << " -- " <<std::endl;
                visualise(gmesh, node, nodes[i]);
                getchar();
            }
            expand(nodes[i], mesh, visibility, level + 1, gmesh, debug);
        }
        delete [] nodes;
    }

    std::vector<Point>
    find_visibility(int edge, const Mesh &mesh, bool side, parsers::GeomMesh &gmesh, bool debig) {
        int num;
        std::vector<Point> r_vis;
        Point tmp;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::get_edge_init_nodes(mesh.mesh_edges[edge], side, mesh, nodes);
        for(int i = 0; i < num; i++){
            if(!side){
                tmp = nodes[i].parent;
                nodes[i].parent = nodes[i].child;
                nodes[i].child = tmp;
            }
            expand(nodes[i], mesh, r_vis, 0, gmesh, debig);
        }
        delete [] nodes;
        return r_vis;
    }
    namespace cgm = cairo_geom_drawer;

    void visualise(parsers::GeomMesh &mesh, SearchNode start, SearchNode expanded){
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        geom::Point<double> vertex, vertex2;
        geom::Point<double> rootP, rootC, edgeR, edgeL, childR, childL;

        for(auto v : mesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : mesh.polygons){
            free.push_back(p.polygon);
        }
        double x_min, x_max, y_min, y_max;
        geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
        geom::Polygon<double> border = {
                {x_min, y_min},
                {x_min, y_max},
                {x_max, y_max},
                {x_max, y_min},
        };
        cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);

        cgm_drawer.OpenPDF("debuging.pdf");
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

        vertex.x = expanded.parent.x;
        vertex.y = expanded.parent.y;
        vertex2.x = expanded.child.x;
        vertex2.y = expanded.child.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorRed, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorRed);

        vertex.x = start.right.x;
        vertex.y = start.right.y;
        vertex2.x = start.left.x;
        vertex2.y = start.left.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorGreen, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorGreen);

        vertex.x = expanded.right.x;
        vertex.y = expanded.right.y;
        vertex2.x = expanded.left.x;
        vertex2.y = expanded.left.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorBlue, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorBlue);

        vertex.x = expanded.parent.x;
        vertex.y = expanded.parent.y;
        vertex2.x = start.left.x;
        vertex2.y = start.left.y;
        vertex2 = vertex + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex);
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorYellow, 0.5);

        vertex2.x = start.right.x;
        vertex2.y = start.right.y;
        vertex.x = expanded.child.x;
        vertex.y = expanded.child.y;
        vertex2 = vertex + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex);
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorYellow, 0.5);
        cgm_drawer.Close();
        return;

    }

}