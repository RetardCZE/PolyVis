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

    void EdgeVisibility::precompute_edges_searchnodes() {
        std::vector<SearchNode> r_v;
        std::vector<SearchNode> l_v;
        int edge = 0;
        for (Edge& e : mesh.mesh_edges){
            r_v.clear(); l_v.clear();
            r_v = this->find_edge_visibility(edge, true);
            l_v = this->find_edge_visibility(edge, false);
            std::cout << l_v.size() << " | " << r_v.size()<<std::endl;
            this->reset_visu();
            std::vector<Point> prop;
            for(auto sn : l_v){

                Point rp, lp;
                if(sn.transitionR.isIntersection) {
                    LineLineIntersectionNotCollinear(mesh.mesh_vertices[sn.transitionR.i.a].p,
                                                     mesh.mesh_vertices[sn.transitionR.i.b].p,
                                                     mesh.mesh_vertices[sn.transitionR.i.c].p,
                                                     mesh.mesh_vertices[sn.transitionR.i.d].p,
                                                     rp);
                }else{
                    rp = mesh.mesh_vertices[sn.transitionR.p].p;
                }
                if(sn.transitionL.isIntersection) {
                    LineLineIntersectionNotCollinear(mesh.mesh_vertices[sn.transitionL.i.a].p,
                                                     mesh.mesh_vertices[sn.transitionL.i.b].p,
                                                     mesh.mesh_vertices[sn.transitionL.i.c].p,
                                                     mesh.mesh_vertices[sn.transitionL.i.d].p,
                                                     lp);
                }else{
                    lp = mesh.mesh_vertices[sn.transitionL.p].p;
                }
                prop.push_back(rp);
                prop.push_back(lp);
                this->visualise_segment(rp, lp, 1, 0.5);

            }
            visualise_polygon(prop,1);
            getchar();
            e.right_nodes.reserve(r_v.size());
            e.left_nodes.reserve(l_v.size());
            e.right_nodes.insert(e.right_nodes.end(), r_v.begin(), r_v.end());
            e.left_nodes.insert(e.left_nodes.end(), l_v.begin(), l_v.end());
            edge++;
        }
    }
    /*
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
                    std::cout << e->left_nodes[i] << std::endl;
                    robustOrientation RP = get_robustOrientation(e->left_nodes[i].P, e->left_nodes[i].root_R, p);
                    robustOrientation LP = get_robustOrientation(e->left_nodes[i].P, e->left_nodes[i].root_L, p);
                    if(RP != robustOrientation::CCW && LP != robustOrientation::CW){
                        if(debug)
                            this->visualise_point(e->left_nodes[i].P, 0);
                        current_state = STATE::VISIBLE;
                    }else{
                        if(debug)
                            this->visualise_point(e->left_nodes[i].P, 2);
                        if(RP == robustOrientation::CCW) {
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
                    std::cout << e->right_nodes[i] << std::endl;
                    robustOrientation RP = get_robustOrientation(e->right_nodes[i].P, e->right_nodes[i].root_R, p);
                    robustOrientation LP = get_robustOrientation(e->right_nodes[i].P, e->right_nodes[i].root_L, p);

                    if(RP != robustOrientation::CCW && LP != robustOrientation::CW){
                        if(debug)
                            this->visualise_point(e->right_nodes[i].P, 0);
                        current_state = STATE::VISIBLE;
                    }else{
                        if(debug)
                            this->visualise_point(e->right_nodes[i].P, 2);
                        if(RP == robustOrientation::CCW) {
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
            if(debug)getchar();
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
    }*/


    void
    EdgeVisibility::expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side) {
        if(node.nextPolygon == -1){
            visibility.push_back(node);
            return;
        }
        int num;

        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];

        num = this->expand_forward(node, nodes);

        for(int i = 0; i < num; i++){
            /*
            this->reset_visu();
            Point lp,rp, lr, rr;
            lp = mesh.mesh_vertices[nodes[i].transitionL.p].p;
            rp= mesh.mesh_vertices[nodes[i].transitionR.p].p;
            lr = mesh.mesh_vertices[nodes[i].rootL.p].p;
            rr= mesh.mesh_vertices[nodes[i].rootR.p].p;
            if(nodes[i].transitionR.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].transitionR.i.a].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionR.i.b].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionR.i.c].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionR.i.d].p,
                                                                                     rp);
            if(nodes[i].transitionL.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].transitionL.i.a].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionL.i.b].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionL.i.c].p,
                                                                                     mesh.mesh_vertices[nodes[i].transitionL.i.d].p,
                                                                                     lp);
            if(nodes[i].rootR.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].rootR.i.a].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.b].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.c].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.d].p,
                                                                               rr);
            if(nodes[i].rootL.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].rootL.i.a].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.b].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.c].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.d].p,
                                                                               lr);

            this->visualise_segment(rp, lp, 1,0.5);
            this->visualise_segment(rr, lr, 0,0.5);
            std::cout << "Pre backward" << std::endl;
            getchar();*/


            this->back_propagation(nodes[i]);
            /*
            this->reset_visu();

            lr = mesh.mesh_vertices[nodes[i].rootL.p].p;
            rr= mesh.mesh_vertices[nodes[i].rootR.p].p;
            if(nodes[i].rootR.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].rootR.i.a].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.b].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.c].p,
                                                                               mesh.mesh_vertices[nodes[i].rootR.i.d].p,
                                                                               rr);
            if(nodes[i].rootL.isIntersection) LineLineIntersectionNotCollinear(mesh.mesh_vertices[nodes[i].rootL.i.a].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.b].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.c].p,
                                                                               mesh.mesh_vertices[nodes[i].rootL.i.d].p,
                                                                               lr);

            this->visualise_segment(rp, lp, 1,0.5);
            this->visualise_segment(rr, lr, 0,0.5);
            getchar();*/

            expand(nodes[i], visibility, side);
        }
        delete [] nodes;
    }

    std::vector<SearchNode>
    EdgeVisibility::find_edge_visibility(int edge, bool side) {
        current_edge = mesh.mesh_edges[edge];
        int num;
        std::vector<SearchNode> vis;

        SearchPoint tmp;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = this->get_edge_init_nodes(mesh.mesh_edges[edge], side, nodes);

        for(int i = 0; i < num; i++){
            expand(nodes[i], vis, side);
        }

        delete [] nodes;
        return vis;
    }

    int
    EdgeVisibility::expand_forward(SearchNode &node, SearchNode *newNodes) {
        // Temporary searchnode object for creating new nodes

        SearchNode temp = init_temp_node(node);

        // right visibility border

        int right_parent, right_child, left_parent, left_child;
        right_child = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        right_parent = node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p;
        // left visibility border
        left_child = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        left_parent = node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p;

        const Polygon &expander = mesh.mesh_polygons[node.nextPolygon];
        std::vector<int> sortedV, sortedP;

        int offset;
        offset = normalise(expander, node.rightVertex, &sortedV, &sortedP);
        const int S = sortedV.size();

        int i;
        uint8_t rCheck = 0;
        uint8_t lCheck = 0;
        // last visible point when going ccw -> left point or cw -> right point
        int right_visible, left_visible;
        int visible = find_visible(node, sortedV, &right_visible, &left_visible);
        // std::cout << left_visible << " | " << right_visible << std::endl;
        SearchPoint right_intersection, left_intersection;
        int count = 0;

        if (Orient(mesh.mesh_vertices[right_parent].p,
                                mesh.mesh_vertices[right_child].p,
                                mesh.mesh_vertices[sortedV[right_visible]].p)
            ==
            robustOrientation::kCollinear) {
            right_intersection.p = sortedV[right_visible];
            right_intersection.isIntersection = false;
        } else {
            right_intersection.isIntersection = true;
            right_intersection.i.is_calculated = false;
            right_intersection.i.a = right_parent;
            right_intersection.i.b = right_child;
            right_intersection.i.c = sortedV[right_visible];
            right_intersection.i.d = sortedV[right_visible-1];
        }

        if (Orient(mesh.mesh_vertices[left_parent].p,
                                mesh.mesh_vertices[left_child].p,
                                mesh.mesh_vertices[sortedV[left_visible]].p)
                                ==
            robustOrientation::kCollinear) {
            left_intersection.isIntersection = false;
        } else {
            left_intersection.isIntersection = true;
            left_intersection.i.is_calculated = false;
            left_intersection.i.a = left_parent;
            left_intersection.i.b = left_child;
            left_intersection.i.c = sortedV[left_visible];
            left_intersection.i.d = sortedV[left_visible+1];
        }


        i = right_visible;
        if(right_intersection.isIntersection) {
            i--;
            temp.transitionR = right_intersection;
        }else{
            temp.transitionR = right_intersection;
        }
        while(i < left_visible){
            temp.transitionL.p = sortedV[i + 1];
            temp.transitionL.isIntersection = false;
            temp.nextPolygon = sortedP[i + 1];
            temp.leftVertex = sortedV[i + 1];
            temp.rightVertex = sortedV[i];
            newNodes[count++] = temp;
            i++;
            temp.transitionR.p = sortedV[i];
            temp.transitionR.isIntersection = false;
        }

        if(left_intersection.isIntersection) {
            temp.transitionL = left_intersection;
            temp.nextPolygon = sortedP[i + 1];
            temp.leftVertex = sortedV[i + 1];
            temp.rightVertex = sortedV[i];
            newNodes[count++] = temp;
        }


        return count;

    }

    int
    EdgeVisibility::find_visible(SearchNode& node, std::vector<int>& sorted_vertices, int* right_visible, int* left_visible){
        const int S = sorted_vertices.size();
        int i;
        // right visibility border
        int right_parent, right_child, left_parent, left_child;
        right_child = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        right_parent = node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p;
        // left visibility border
        left_child = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        left_parent = node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p;
        // last visible point when going ccw -> left point or cw -> right point
        *right_visible = S-1;
        *left_visible = 0;
        for(i = 0; i<S; i++){
            if (Orient(mesh.mesh_vertices[right_parent].p,
                                    mesh.mesh_vertices[right_child].p,
                                    mesh.mesh_vertices[sorted_vertices[S-1-i]].p) == robustOrientation::kRightTurn){
                if( i == 0)
                    std::cerr << "WARNING POLYGON NOT VISIBLE - RIGHT" << std::endl;
                break;
            }
            *right_visible = S-1-i;
        }
        for(i = 0; i<S; i++){
            if (Orient(mesh.mesh_vertices[left_parent].p,
                                    mesh.mesh_vertices[left_child].p,
                                    mesh.mesh_vertices[sorted_vertices[i]].p) == robustOrientation::kLeftTurn){
                if( i == 0)
                    std::cerr << "WARNING POLYGON NOT VISIBLE - LEFT" << std::endl;
                break;
            }
            *left_visible = i;
        }
        return *left_visible - *right_visible;
    }

    void
    EdgeVisibility::back_propagation(edgevis::SearchNode &node) {
        /*
         * Be careful about robustOrientation of intersection lines!
         */
        SearchPoint parent_intersection, child_intersection, temp;
        SearchNode* parent;
        parent = node.predecessor;

        int right_parent, right_child, left_parent, left_child;
        if(node.transitionR.isIntersection){
            /*
             * It has same root as previous node.
             */
        }
        else{
            right_parent = node.transitionR.p;
            while(parent){
                if(parent->transitionL.isIntersection) {
                    parent = parent->predecessor;
                    continue;
                }
                right_child = parent->transitionL.p;

                if(node.rootL.isIntersection){
                    if(Orient(mesh.mesh_vertices[right_parent].p,
                                           mesh.mesh_vertices[node.rootL.i.a].p,
                                           mesh.mesh_vertices[right_child].p)
                       ==
                       robustOrientation::kLeftTurn){
                        node.rootL.isIntersection = true;
                        node.rootL.i.a = right_child;
                        node.rootL.i.b = right_parent;
                        node.rootL.i.c = current_edge.parent;
                        node.rootL.i.d = current_edge.child;
                    }
                }
                else{
                    if(Orient(mesh.mesh_vertices[right_parent].p,
                                        mesh.mesh_vertices[right_child].p,
                                        mesh.mesh_vertices[node.rootL.p].p)
                       ==
                       robustOrientation::kRightTurn){
                        node.rootL.isIntersection = true;
                        node.rootL.i.a = right_child;
                        node.rootL.i.b = right_parent;
                        node.rootL.i.c = current_edge.parent;
                        node.rootL.i.d = current_edge.child;

                    }
                }
                parent = parent->predecessor;
            }
        }

        parent = node.predecessor;
        if(node.transitionL.isIntersection){
            /*
             * It has same root as previous node.
             */
        }else{
            left_parent = node.transitionL.p;
            while(parent){
                if(parent->transitionR.isIntersection) {
                    parent = parent->predecessor;
                    continue;
                }
                left_child = parent->transitionR.p;

                if(node.rootR.isIntersection){
                    if(Orient(mesh.mesh_vertices[left_parent].p,
                                           mesh.mesh_vertices[node.rootR.i.a].p,
                                           mesh.mesh_vertices[left_child].p)
                       ==
                       robustOrientation::kRightTurn){
                        node.rootR.isIntersection = true;
                        node.rootR.i.a = left_child;
                        node.rootR.i.b = left_parent;
                        node.rootR.i.c = current_edge.parent;
                        node.rootR.i.d = current_edge.child;
                    }
                }
                else{
                    if(Orient(mesh.mesh_vertices[left_parent].p,
                                           mesh.mesh_vertices[left_child].p,
                                           mesh.mesh_vertices[node.rootR.p].p)
                       ==
                       robustOrientation::kLeftTurn){
                        node.rootR.isIntersection = true;
                        node.rootR.i.a = left_child;
                        node.rootR.i.b = left_parent;
                        node.rootR.i.c = current_edge.parent;
                        node.rootR.i.d = current_edge.child;

                    }
                }
                parent = parent->predecessor;
            }
        }
        return;
    }


    int
    EdgeVisibility::get_edge_init_nodes(Edge edge, bool side, edgevis::SearchNode *initNodes) {
        // side - true = transitionR, false = transitionL
        Polygon expander;
        SearchNode temp;
        temp.predecessor = NULL;
        temp.rootL.isIntersection = false;
        temp.rootR.isIntersection = false;
        temp.transitionR.isIntersection = false;
        temp.transitionL.isIntersection = false;
        int count = 0;
        std::vector<int> sortedV, sortedP;

        if(side) {
            if(edge.rightPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.rightPoly];
            normalise(expander, edge.parent, &sortedV, &sortedP);
            temp.rootR.p = edge.parent;
            temp.rootL.p = edge.child;
            temp.comingFrom = edge.rightPoly;
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.leftPoly];
            normalise(expander, edge.child, &sortedV, &sortedP);
            temp.rootR.p = edge.child;
            temp.rootL.p = edge.parent;
            temp.comingFrom = edge.leftPoly;
        }

        for(int i = 0; i < sortedV.size()-1; i++){
            temp.transitionR.p = sortedV[i];
            temp.transitionL.p = sortedV[i + 1];
            temp.rightVertex = sortedV[i];
            temp.leftVertex = sortedV[i + 1];
            temp.nextPolygon = sortedP[i + 1];
            initNodes[count++] = temp;
        }

        return count;
    }





    /*
     *
     *
     *
     *          VISIBILITY
     *
     *
     *
     */


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
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
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