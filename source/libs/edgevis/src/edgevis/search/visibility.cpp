#include "edgevis/search/visibility.h"

namespace edgevis{

    EdgeVisibility::EdgeVisibility(Mesh& mesh) {
        this->mesh = mesh;
    }

    EdgeVisibility::~EdgeVisibility(){
        cgm_drawer.Close();
    }

    bool
    EdgeVisibility::switch_debug(bool on) {
        this->debug = on;
        return on;
    }

    const Mesh& EdgeVisibility::mesh_reference() {
        return mesh;

    }

    void EdgeVisibility::precompute_edges() {
        std::vector<Point> r_v;
        std::vector<Point> l_v;
        int edge = 0;
        for (Edge& e : mesh.mesh_edges){
            r_v.clear(); l_v.clear();
            r_v = this->find_visibility(edge, true);
            l_v = this->find_visibility(edge, false);
            e.right_visibility.reserve(r_v.size());
            e.left_visibility.reserve(l_v.size());
            e.right_visibility.insert(e.right_visibility.end(), r_v.begin(), r_v.end());
            e.left_visibility.insert(e.left_visibility.end(), l_v.begin(), l_v.end());
            edge++;
        }
    }

    std::vector<Point>
    EdgeVisibility::find_point_visibility(Point p) {
        std::vector<Point> v;
        Point left, right, vector;
        std::vector<Edge*> edges = this->get_init_edges(p);
        if(edges.size() == 0)
            return v;
        for (Edge* e :edges){
            this->reset_visu();
            vector = mesh.mesh_vertices[e->child].p - p;
            left = p + vector;
            while(mesh.min_x < left.x && left.x < mesh.max_x && mesh.min_y < left.y && left.y < mesh.max_y){
                left = left + vector;
            }
            this->visualise_segment(p, left, 2);
            vector = (mesh.mesh_vertices[e->parent].p - p);
            right = p + vector;
            while(mesh.min_x < right.x && right.x < mesh.max_x && mesh.min_y < right.y & right.y < mesh.max_y){
                right = right + vector;
            }
            this->visualise_segment(p, right, 2);
            this->visualise_point(p, 0);
            this->visualise_segment(mesh.mesh_vertices[e->parent].p, mesh.mesh_vertices[e->child].p, 1);
            this->visualise_polygon(e->right_visibility, 2);
            getchar();
        }
        return v;
    }

    std::vector<Edge*> EdgeVisibility::get_init_edges(Point p) {
        std::vector<Edge*> edges;
        PointLocation pl = mesh.get_point_location(p);
        std::cout << p << " | " << pl << std::endl;
        switch(pl.type){
            case PointLocation::NOT_ON_MESH:
                edges.clear();
                return edges;
            case PointLocation::IN_POLYGON:
                for( auto e : mesh.mesh_polygons[pl.poly1].edges ){
                    edges.push_back(&mesh.mesh_edges[e]);
                }
                break;
            case PointLocation::ON_MESH_BORDER:
            case PointLocation::ON_EDGE:
                break;
            case PointLocation::ON_CORNER_VERTEX_AMBIG:
            case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
            case PointLocation::ON_NON_CORNER_VERTEX:

                break;
        }
        return edges;
    }


    void
    EdgeVisibility::expand(SearchNode &node, std::vector<Point> &visibility, int level) {
        if(node.next_polygon == -1){
            visibility.push_back(node.child_R);
            visibility.push_back(node.child_L);
            return;
        }
        int num;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::expand_searchnode(node, mesh, nodes);

        for(int i = 0; i < num; i++){
            recompute_roots(nodes[i]);
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
    void
    EdgeVisibility::set_visual_mesh(const parsers::GeomMesh &gmesh) {
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;

        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
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

        cgm_drawer.OpenImage();
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

    }

    void EdgeVisibility::reset_visu(){
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        cgm_drawer.Close();
        cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 100);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
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
        cgm_drawer.OpenImage();
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);
    }

    void
    EdgeVisibility::visualise_segment(Point A, Point B, int color){
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x;
        vertex.y = A.y;
        vertex2.x = B.x;
        vertex2.y = B.y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_point(Point A, int color){
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x;
        vertex.y = A.y;
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_polygon(std::vector<Point>& p, int color) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        cgm::RGB light_colors[3] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x;
            vertex.y = v.y;
            polygon.push_back(vertex);
        }

        cgm_drawer.DrawPolygon(polygon, light_colors[color], 0.5);
        cgm_drawer.DrawPoints(polygon, 0.2, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

}