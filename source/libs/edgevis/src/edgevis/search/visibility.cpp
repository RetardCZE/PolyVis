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
        std::vector<OptimNode> r_v;
        std::vector<OptimNode> l_v;
        int edge = 0;
        for (Edge& e : mesh.mesh_edges){
            r_v.clear(); l_v.clear();
            r_v = this->find_visibility(edge, true);
            l_v = this->find_visibility(edge, false);
            e.right_nodes.reserve(r_v.size());
            e.left_nodes.reserve(l_v.size());
            e.right_nodes.insert(e.right_nodes.end(), r_v.begin(), r_v.end());
            e.left_nodes.insert(e.left_nodes.end(), l_v.begin(), l_v.end());
            edge++;
        }
    }

    std::vector<Point>
    EdgeVisibility::find_point_visibility(Point p, std::vector<Point> visu, bool debug) {
        std::vector<Point> out;
        PointLocation pl = mesh.get_point_location(p);
        std::vector<Edge*> edges = this->get_init_edges(pl);
        std::string saving;
        if(edges.size() == 0) {
            out.clear();
            return out;
        }
        STATE last_state = STATE::VISIBLE;
        STATE current_state;
        Point last_visible;
        Point last_point;
        Point segment_holder[2];
        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->left_nodes.size() > 0) {
                for(int i = 0; i < e->left_nodes.size(); i++){
                    Orientation RP = get_orientation(e->left_nodes[i].P, e->left_nodes[i].root_R, p);
                    Orientation LP = get_orientation(e->left_nodes[i].P, e->left_nodes[i].root_L, p);
                    if(RP != Orientation::CCW && LP != Orientation::CW){
                        if(debug)
                            this->visualise_point(e->left_nodes[i].P, 0);
                        current_state = STATE::VISIBLE;
                    }else{
                        if(debug)
                            this->visualise_point(e->left_nodes[i].P, 2);
                        if(RP == Orientation::CCW) {
                            current_state = STATE::RIGHT;
                        }else{
                            current_state = STATE::LEFT;
                        }
                    }
                    if(current_state != last_state){
                        if(last_state == STATE::RIGHT){
                            // leaving RIGHT causes triggering of intersection
                            Point I = line_intersect(p, last_visible, last_point, e->left_nodes[i].P);
                            if(debug)
                                this->visualise_point(I, 1);
                            out.push_back(I);
                        }
                        if(current_state == STATE::LEFT){
                            // entering left causes hanging intersection
                            segment_holder[0] = last_point;
                            segment_holder[1] = e->left_nodes[i].P;
                        }
                        if(last_state == STATE::LEFT && current_state == STATE::VISIBLE){
                            // leaving LEFT will trigger intersection with segment in memory
                            Point I = line_intersect(p, e->left_nodes[i].P, segment_holder[0], segment_holder[1]);
                            if(debug)
                                this->visualise_point(I, 1);
                            out.push_back(I);
                        }
                    }

                    if(current_state == STATE::VISIBLE){
                        last_visible = e->left_nodes[i].P;
                        out.push_back(e->left_nodes[i].P);
                    }
                    last_point = e->left_nodes[i].P;
                    last_state = current_state;
                }
            }

            if(pl.poly1 == e->leftPoly && e->right_nodes.size()>0){
                for(int i = 0; i < e->right_nodes.size(); i++) {
                    Orientation RP = get_orientation(e->right_nodes[i].P, e->right_nodes[i].root_R, p);
                    Orientation LP = get_orientation(e->right_nodes[i].P, e->right_nodes[i].root_L, p);

                    if(RP != Orientation::CCW && LP != Orientation::CW){
                        if(debug)
                            this->visualise_point(e->right_nodes[i].P, 0);
                        current_state = STATE::VISIBLE;
                    }else{
                        if(debug)
                            this->visualise_point(e->right_nodes[i].P, 2);
                        if(RP == Orientation::CCW) {
                            current_state = STATE::RIGHT;
                        }else{
                            current_state = STATE::LEFT;
                        }
                    }
                    if(current_state != last_state){
                        if(last_state == STATE::RIGHT){
                            // leaving RIGHT causes triggering of intersection
                            Point I = line_intersect(p, last_visible, last_point, e->right_nodes[i].P);
                            if(debug)
                                this->visualise_point(I, 1);
                            out.push_back(I);
                        }
                        if(current_state == STATE::LEFT){
                            // entering left causes hanging intersection
                            segment_holder[0] = last_point;
                            segment_holder[1] = e->right_nodes[i].P;
                        }
                        if(last_state == STATE::LEFT && current_state == STATE::VISIBLE){
                            // leaving LEFT will trigger intersection with segment in memory
                            Point I = line_intersect(p, e->right_nodes[i].P, segment_holder[0], segment_holder[1]);
                            if(debug)
                                this->visualise_point(I, 1);
                            out.push_back(I);
                        }
                    }

                    if(current_state == STATE::VISIBLE){
                        last_visible = e->right_nodes[i].P;
                        out.push_back(e->right_nodes[i].P);
                    }
                    last_point = e->right_nodes[i].P;
                    last_state = current_state;
                }
            }
        }
        return out;
    }

    std::vector<Edge*> EdgeVisibility::get_init_edges(PointLocation pl) {
        std::vector<Edge*> edges;
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
    EdgeVisibility::expand(SearchNode &node, std::vector<OptimNode> &visibility, int level, bool side) {
        OptimNode o;
        Point true_root_R;
        Point true_root_L;
        if(side){
            true_root_R = mesh.mesh_vertices[current_edge.parent].p;
            true_root_L = mesh.mesh_vertices[current_edge.child].p;
        }else{
            true_root_L = mesh.mesh_vertices[current_edge.parent].p;
            true_root_R = mesh.mesh_vertices[current_edge.child].p;
        }
        if(node.next_polygon == -1){
            o.P = node.child_R;
            o.pivot_R = true_root_R;
            o.pivot_L = true_root_L;
            o.root_R = true_root_R;
            o.root_L =  true_root_L;
            recompute_end_roots(node, o);
            visibility.push_back(o);


            o.P = node.child_L;
            o.pivot_R = true_root_R;
            o.pivot_L = true_root_L;
            o.root_R = true_root_R;
            o.root_L =  true_root_L;
            recompute_end_roots(node, o);
            visibility.push_back(o);

            return;
        }
        int num;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::expand_searchnode(node, mesh, nodes);

        for(int i = 0; i < num; i++){
            recompute_roots(nodes[i]);
            expand(nodes[i], visibility, level + 1, side);
        }
        delete [] nodes;
    }

    std::vector<OptimNode>
    EdgeVisibility::find_visibility(int edge, bool side) {
        current_edge = mesh.mesh_edges[edge];
        int num;
        std::vector<OptimNode> vis;

        Point tmp;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::get_edge_init_nodes(mesh.mesh_edges[edge], side, mesh, nodes);
        OptimNode o;
        if(side){
            o.P = mesh.mesh_vertices[current_edge.parent].p;
            o.pivot_R = mesh.mesh_vertices[current_edge.parent].p;
            o.pivot_L = mesh.mesh_vertices[current_edge.child].p;
            o.root_R = mesh.mesh_vertices[current_edge.parent].p;
            o.root_L =  mesh.mesh_vertices[current_edge.child].p;
        }else{
            o.P = mesh.mesh_vertices[current_edge.child].p;
            o.pivot_R = mesh.mesh_vertices[current_edge.child].p;
            o.pivot_L = mesh.mesh_vertices[current_edge.parent].p;
            o.root_R = mesh.mesh_vertices[current_edge.child].p;
            o.root_L =  mesh.mesh_vertices[current_edge.parent].p;
        }
        vis.push_back(o);

        for(int i = 0; i < num; i++){
            if(!side){
                tmp = nodes[i].root_R;
                nodes[i].root_R = nodes[i].root_L;
                nodes[i].root_L = tmp;
            }
            expand(nodes[i], vis, 0, side);
        }

        if(side){
            o.P = mesh.mesh_vertices[current_edge.child].p;
            o.pivot_R = mesh.mesh_vertices[current_edge.parent].p;
            o.pivot_L = mesh.mesh_vertices[current_edge.child].p;
            o.root_R = mesh.mesh_vertices[current_edge.parent].p;
            o.root_L =  mesh.mesh_vertices[current_edge.child].p;
        }else{
            o.P = mesh.mesh_vertices[current_edge.parent].p;
            o.pivot_R = mesh.mesh_vertices[current_edge.child].p;
            o.pivot_L = mesh.mesh_vertices[current_edge.parent].p;
            o.root_R = mesh.mesh_vertices[current_edge.child].p;
            o.root_L =  mesh.mesh_vertices[current_edge.parent].p;
        }
        vis.push_back(o);


        delete [] nodes;
        return vis;
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
        cgm_drawer = cgm::CairoGeomDrawer(this->mesh.max_x, this->mesh.max_y, 20);
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
    EdgeVisibility::visualise_segment(Point A, Point B, int color, float opacity) {
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
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_polygon(std::vector<Point>& p, int color) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorYellow};
        cgm::RGB light_colors[4] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue, cgm::kColorLightYellow};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x;
            vertex.y = v.y;
            polygon.push_back(vertex);
        }

        cgm_drawer.DrawPolygon(polygon, light_colors[color], 0.5);
        //cgm_drawer.DrawPoints(polygon, 0.15, colors[color], 0.5);

        for( int i = 0; i < polygon.size() - 1; i++){
            cgm_drawer.DrawLine(polygon[i], polygon[i+1], 0.1, colors[color], 0.3);
        }
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

}