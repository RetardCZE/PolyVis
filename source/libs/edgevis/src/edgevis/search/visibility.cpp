#include "edgevis/search/visibility.h"

namespace edgevis{

    EdgeVisibility::EdgeVisibility(const Mesh& mesh) {
        this->mesh = mesh;
    }

    EdgeVisibility::~EdgeVisibility(){

    }

    void
    EdgeVisibility::set_visual_mesh(const parsers::GeomMesh &gmesh) {
        this->gmesh = gmesh;
    }

    bool
    EdgeVisibility::switch_debug(bool on) {
        this->debug = on;
        return on;
    }

    void
    EdgeVisibility::expand(SearchNode &node, std::vector<Point> &visibility, int level) {
        if(debug)
        std::cout << node  << "\n";

        if(node.next_polygon == -1){
            visibility.push_back(node.child_R);
            visibility.push_back(node.child_L);
            if(debug)
            std::cout << node.child_L << " | " << node.child_R << std::endl;
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
            recompute_roots(nodes[i]);
            if(debug) {std::cout << nodes[i] << " -- " <<std::endl;
                visualise(gmesh, node, nodes[i]);
                getchar();
            }
            expand(nodes[i], visibility, level + 1);
        }
        delete [] nodes;
    }

    std::vector<Point>
    EdgeVisibility::find_visibility(int edge, bool side) {
        int num;
        std::vector<Point> r_vis;
        Point tmp;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::get_edge_init_nodes(mesh.mesh_edges[edge], side, mesh, nodes);
        if(debug){
            for(int i = 0; i < num; i++){
                visualise(gmesh, nodes[i], nodes[i]);std::cout << nodes[i];
                getchar();

            }
        }
        for(int i = 0; i < num; i++){
            if(!side){
                tmp = nodes[i].root_R;
                nodes[i].root_R = nodes[i].root_L;
                nodes[i].root_L = tmp;
            }
            expand(nodes[i], r_vis, 0);
        }
        delete [] nodes;
        return r_vis;
    }
    namespace cgm = cairo_geom_drawer;

    void
    EdgeVisibility::visualise(parsers::GeomMesh &mesh, SearchNode start, SearchNode expanded){
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

        vertex.x = expanded.root_R.x;
        vertex.y = expanded.root_R.y;
        vertex2.x = expanded.root_L.x;
        vertex2.y = expanded.root_L.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorRed, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorRed);

        vertex.x = start.child_R.x;
        vertex.y = start.child_R.y;
        vertex2.x = start.child_L.x;
        vertex2.y = start.child_L.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorGreen, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorGreen);

        vertex.x = expanded.child_R.x;
        vertex.y = expanded.child_R.y;
        vertex2.x = expanded.child_L.x;
        vertex2.y = expanded.child_L.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorBlue, 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorBlue);

        vertex.x = expanded.root_R.x;
        vertex.y = expanded.root_R.y;
        vertex2.x = start.child_L.x;
        vertex2.y = start.child_L.y;
        vertex2 = vertex + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex);
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorYellow, 0.5);

        vertex2.x = start.child_R.x;
        vertex2.y = start.child_R.y;
        vertex.x = expanded.root_L.x;
        vertex.y = expanded.root_L.y;
        vertex2 = vertex + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex) + (vertex2 - vertex);
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorYellow, 0.5);
        cgm_drawer.Close();
        return;

    }

}