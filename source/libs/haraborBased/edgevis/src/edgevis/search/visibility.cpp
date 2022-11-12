#include "edgevis/search/visibility.h"

namespace edgevis{

    EdgeVisibility::EdgeVisibility(Mesh& mesh) {
        this->mesh = mesh;
    }

    EdgeVisibility::~EdgeVisibility(){
        cgm_drawer.Close();
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
            e.right_nodes.reserve(r_v.size());
            e.left_nodes.reserve(l_v.size());
            e.right_nodes.insert(e.right_nodes.end(), r_v.begin(), r_v.end());
            e.left_nodes.insert(e.left_nodes.end(), l_v.begin(), l_v.end());
            edge++;
        }
        searchnodes_precomputed = true;
    }

    std::vector<OptimNodeV1>
    EdgeVisibility::compute_side_optimnodesV1(Edge &edge, bool right){
        std::vector<OptimNodeV1> v;
        OptimNodeV1 on1, on2;

        on1.P.isIntersection = false;
        if(right) {
            on1.P.p = edge.parent;
            on1.root_R.p = edge.parent;
            on1.root_L.p = edge.child;
        }else{
            on1.P.p = edge.child;
            on1.root_R.p = edge.child;
            on1.root_L.p = edge.parent;
        }
        on1.root_R.isIntersection = false;
        on1.root_L.isIntersection = false;
        v.push_back(on1);
        std::vector<SearchNode>* vec = right ? &edge.right_nodes : &edge.left_nodes;
        if(vec->size() > 0) {
            for (auto sn: *vec) {
                this->compute_optimnodesv1(sn, on1, on2);

                if (on1 != v.back()) {
                    v.push_back(on1);
                }
                if (on2 != v.back()) {
                    v.push_back(on2);
                }
            }
        }

        on1.P.isIntersection = false;
        if(right) {
            on1.P.p = edge.child;
            on1.root_R.p = edge.parent;
            on1.root_L.p = edge.child;
        }else{
            on1.P.p = edge.parent;
            on1.root_R.p = edge.child;
            on1.root_L.p = edge.parent;
        }
        on1.root_R.isIntersection = false;
        on1.root_L.isIntersection = false;
        v.push_back(on1);
        return v;
    }

    void EdgeVisibility::precompute_edges_optimnodesV1() {
        if(!searchnodes_precomputed){
            this->precompute_edges_searchnodes();
        }
        std::vector<OptimNodeV1> r_v;
        std::vector<OptimNodeV1> l_v;
        OptimNodeV1 on1, on2;
        SearchNode sn;
        int edge = 0;
        for (Edge& e : mesh.mesh_edges){
            r_v = compute_side_optimnodesV1(e, true);
            l_v = compute_side_optimnodesV1(e, false);
            e.rightOptimNodesV1.reserve(r_v.size());
            e.leftOptimNodesV1.reserve(l_v.size());
            e.rightOptimNodesV1.insert(e.rightOptimNodesV1.end(), r_v.begin(), r_v.end());
            e.leftOptimNodesV1.insert(e.leftOptimNodesV1.end(), l_v.begin(), l_v.end());
            edge++;
        }
    }

    void
    EdgeVisibility::compute_optimnodesv1(SearchNode &node, OptimNodeV1 &o1, OptimNodeV1 &o2) {
        SearchNode* parent;
        o1.P = node.transitionR;
        o1.root_R.isIntersection = false;
        o1.root_R.p = node.rightRootVertex;
        o1.root_L = node.rootL;

        o2.P = node.transitionL;
        o2.root_R = node.rootR;
        o2.root_L.isIntersection = false;
        o2.root_L.p = node.leftRootVertex;
        parent = node.predecessor;
        while (parent) {
            if(o1.P.isIntersection) {
                o1.root_R = node.rootL;
            }else if(!parent->transitionR.isIntersection) {
                int root_dir = o1.root_R.isIntersection ? o1.root_R.i.a : o1.root_R.p;
                if(Orient(mesh.mesh_vertices[o1.P.p].p,
                          mesh.mesh_vertices[root_dir].p,
                          mesh.mesh_vertices[parent->transitionR.p].p)
                    ==
                    robustOrientation::kRightTurn){
                     o1.root_R.isIntersection = true;
                     o1.root_R.i.b = o1.P.p;
                     o1.root_R.i.a = parent->transitionR.p;
                     o1.root_R.i.c = node.rightRootVertex;
                     o1.root_R.i.d = node.leftRootVertex;
                }
            }

            if(o2.P.isIntersection) {
                o2.root_L = node.rootR;
            }else if(!parent->transitionL.isIntersection){
                int root_dir = o2.root_L.isIntersection ? o2.root_L.i.a : o2.root_L.p;
                if(Orient(mesh.mesh_vertices[o2.P.p].p,
                          mesh.mesh_vertices[root_dir].p,
                          mesh.mesh_vertices[parent->transitionL.p].p)
                   ==
                   robustOrientation::kLeftTurn){
                    o2.root_L.isIntersection = true;
                    o2.root_L.i.b = o2.P.p;
                    o2.root_L.i.a = parent->transitionL.p;
                    o2.root_L.i.c = node.rightRootVertex;
                    o2.root_L.i.d = node.leftRootVertex;
                }
            }
            parent = parent->predecessor;
        }
        return;
    }

    Point
    EdgeVisibility::evaluate_intersection(SearchPoint& sp){
        if(!sp.isIntersection){
            return mesh.mesh_vertices[sp.p].p;
        }else{
            Point p;
            LineLineIntersectionNotCollinear(mesh.mesh_vertices[sp.i.a].p,
                                             mesh.mesh_vertices[sp.i.b].p,
                                             mesh.mesh_vertices[sp.i.c].p,
                                             mesh.mesh_vertices[sp.i.d].p,
                                             p);
            return p;
        }
    }

    std::vector<Point>
    EdgeVisibility::find_point_visibility_optim1(Point p, bool debug, double &steps) {
        steps = 0;
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
        int left_parent, left_child, right_parent, right_child;
        Point last_visible;
        Point last_point;
        Point I;
        Point segment_holder[2];
        std::vector<OptimNodeV1> *vec;
        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->leftOptimNodesV1.size() > 0) {
                vec = &e->leftOptimNodesV1;
            }else if((pl.poly1 == e->leftPoly && e->rightOptimNodesV1.size() > 0)) {
                vec = &e->rightOptimNodesV1;
            }
            for(auto node : *vec){
                steps++;
                if(node.root_L.isIntersection){
                    left_parent = node.root_L.i.a;
                    left_child = node.P.isIntersection ? node.root_L.i.b : node.P.p;
                }else{
                    left_parent = node.root_L.p;
                    left_child = node.P.isIntersection ? node.P.i.b : node.P.p;
                }
                if(node.root_R.isIntersection){
                    right_parent = node.root_R.i.a;
                    right_child = node.P.isIntersection ? node.root_R.i.b : node.P.p;
                }else{
                    right_parent = node.root_R.p;
                    right_child = node.P.isIntersection ? node.P.i.b : node.P.p;
                }
                robustOrientation rightOri = Orient(mesh.mesh_vertices[right_child].p,
                                                    mesh.mesh_vertices[right_parent].p,
                                                    p);
                robustOrientation leftOri = Orient(mesh.mesh_vertices[left_child].p,
                                                   mesh.mesh_vertices[left_parent].p,
                                                   p);
                if(rightOri != robustOrientation::kLeftTurn && leftOri != robustOrientation::kRightTurn){
                    current_state = STATE::VISIBLE;
                }else{
                    if(rightOri == robustOrientation::kLeftTurn) {
                        current_state = STATE::RIGHT;
                    }else{
                        current_state = STATE::LEFT;
                    }
                }
                if(current_state != last_state){
                    if(last_state == STATE::RIGHT){
                        // leaving RIGHT causes triggering of intersection
                        LineLineIntersectionNotCollinear(p, last_visible, last_point, this->evaluate_intersection(node.P), I);
                        out.push_back(I);
                    }
                    if(current_state == STATE::LEFT){
                        // entering left causes hanging intersection
                        segment_holder[0] = last_point;
                        segment_holder[1] = this->evaluate_intersection(node.P);
                    }
                    if(last_state == STATE::LEFT && current_state == STATE::VISIBLE){
                        // leaving LEFT will trigger intersection with segment in memory
                        LineLineIntersectionNotCollinear(p, this->evaluate_intersection(node.P), segment_holder[0], segment_holder[1], I);
                        out.push_back(I);
                    }
                }

                if(current_state == STATE::VISIBLE){
                    last_visible = this->evaluate_intersection(node.P);
                    out.push_back(last_visible);
                }
                last_point = this->evaluate_intersection(node.P);
                last_state = current_state;
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
    EdgeVisibility::expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side) {
        if(node.nextPolygon == -1){
            visibility.push_back(node);
            return;
        }
        int num;

        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];

        num = this->expand_forward(node, nodes);

        for(int i = 0; i < num; i++){
            this->back_propagation(nodes[i]);
            expand(nodes[i], visibility, side);
        }
        return;
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

        //delete [] nodes;
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
                if(!parent->transitionL.isIntersection) {
                    right_child = parent->transitionL.p;
                    int root_dir = node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p;
                    robustOrientation ori = Orient(mesh.mesh_vertices[right_parent].p,
                                                  mesh.mesh_vertices[root_dir].p,
                                                  mesh.mesh_vertices[right_child].p);
                    if (ori == robustOrientation::kLeftTurn or ori ==robustOrientation::kCollinear) {
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
                if(!parent->transitionR.isIntersection) {
                    left_child = parent->transitionR.p;

                    int root_dir = node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p;
                    robustOrientation ori = Orient(mesh.mesh_vertices[left_parent].p,
                                                   mesh.mesh_vertices[root_dir].p,
                                                   mesh.mesh_vertices[left_child].p);
                    if(ori == robustOrientation::kRightTurn or ori == robustOrientation::kCollinear){
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
            temp.rightRootVertex = edge.parent;
            temp.leftRootVertex = edge.child;
            temp.comingFrom = edge.rightPoly;
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.leftPoly];
            normalise(expander, edge.child, &sortedV, &sortedP);
            temp.rootR.p = edge.child;
            temp.rootL.p = edge.parent;
            temp.rightRootVertex = edge.child;
            temp.leftRootVertex = edge.parent;
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


    void
    EdgeVisibility::find_arbitrary_edge_visibility(int edgeParent, int edgeChild, std::vector<SearchNode> &leftVis,
                                                   std::vector<SearchNode> &rightVis) {
        Edge e;
        e.parent = edgeParent;
        e.child = edgeChild;
        e.leftPoly = e.rightPoly = NULL;
        int numLeft;
        int numRight;
        this->visualise_segment(mesh.mesh_vertices[edgeParent].p,
                                mesh.mesh_vertices[edgeChild].p,
                               0, 0.5);

        SearchNode* leftNodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        SearchNode* rightNodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        this->get_arbitrary_edge_init_nodes(e, numRight, numLeft, leftNodes, rightNodes);

        std::vector<Point> lPoints;
        if(numLeft){
            for(int i = 0; i < numLeft; i++){
                expand(leftNodes[i], leftVis, false); // here side should be deprecated
            }
        }
        if(numRight){
            for(int i = 0; i < numRight; i++){
                expand(rightNodes[i], rightVis, true); // here side should be deprecated
            }
        }
        return;
    }


    void
    EdgeVisibility::get_arbitrary_edge_init_nodes(Edge edge, int &rightCount, int &leftCount,
                                                  edgevis::SearchNode *initLeftNodes,
                                                  edgevis::SearchNode *initRightNodes) {
        rightCount = 0;
        leftCount = 0;
        // side - true = transitionR, false = transitionL
        std::vector<Point> verticesLeft, verticesRight;
        std::vector<int> edges, tmp_edges;
        edges.clear();
        tmp_edges.clear();

        bool show, is_used;
        int new_p;
        std::vector<int> polygons, used;

        for(auto p :  mesh.mesh_vertices[edge.parent].polygons) {
            polygons.push_back(p);
        }

        while(polygons.size()){
            int p = polygons.front();
            used.push_back(p);
            polygons.erase(polygons.begin());
            if(p == -1) continue;

            show = false;
            for(auto e : mesh.mesh_polygons[p].edges){
                if(mesh.mesh_edges[e] == edge) continue; // non oriented equality
                Point point;
                uint8_t check = SegmentSegmentIntersectionGeneral(mesh.mesh_vertices[edge.parent].p,
                                                                  mesh.mesh_vertices[edge.child].p,
                                                                  mesh.mesh_vertices[mesh.mesh_edges[e].parent].p,
                                                                  mesh.mesh_vertices[mesh.mesh_edges[e].child].p,
                                                                  point);
                if(check==0){
                    show = true;
                    is_used = false;
                    if(mesh.mesh_edges[e].leftPoly == p){
                        new_p = mesh.mesh_edges[e].rightPoly;
                    }else{
                        new_p = mesh.mesh_edges[e].leftPoly;
                    }
                    for(auto u : used){
                        if(new_p == u){
                            is_used = true;
                            break;
                        }
                    }
                    if(!is_used && new_p != -1){
                        polygons.push_back(new_p);
                    }

                }else if(check == 2 && point != mesh.mesh_vertices[edge.parent].p){
                    show = true;
                    if(point != mesh.mesh_vertices[edge.child].p){
                        for(auto new_p : mesh.mesh_vertices[e].polygons){
                            is_used = false;
                            for(auto u : used){
                                if(new_p == u){
                                    is_used = true;
                                    break;
                                }
                            }
                            if(!is_used && new_p != -1){
                                polygons.push_back(new_p);
                            }
                        }
                    }else{
                        tmp_edges.push_back(e);
                    }
                }else{
                    tmp_edges.push_back(e);
                }
            }
            if(show){
                for(auto t: tmp_edges) edges.push_back(t);
            }
            tmp_edges.clear();
        }
        /*
         * for(auto e : edges){
         *     this->visualise_segment(mesh.mesh_vertices[mesh.mesh_edges[e].parent].p,
         *                             mesh.mesh_vertices[mesh.mesh_edges[e].child].p,
         *                             1, 0.5);
         * }
         * getchar();
         */
        int current_edge = edges.front();
        int p,c, beforeP;
        int current_point = mesh.mesh_edges[current_edge].child;
        std::vector<int> orderedVerts, orderedLeftPoly, orderedRightPoly;
        edges.erase(edges.begin());
        orderedVerts.push_back(current_point);
        orderedLeftPoly.push_back(mesh.mesh_edges[current_edge].leftPoly);
        orderedRightPoly.push_back(mesh.mesh_edges[current_edge].rightPoly);
        bool disconnected;
        while(edges.size()){
            if(current_point==edge.parent){
                p = orderedVerts.size() - 1;
            }
            if(current_point==edge.child){
                c = orderedVerts.size() - 1;
            }
            disconnected = true;
            for(int i = 0; i<edges.size(); i++){
                if(current_point == mesh.mesh_edges[edges[i]].parent){
                    current_edge = edges[i];
                    current_point = mesh.mesh_edges[current_edge].child;
                    orderedVerts.push_back(current_point);
                    edges.erase(edges.begin()+i);
                    orderedLeftPoly.push_back(mesh.mesh_edges[current_edge].leftPoly);
                    orderedRightPoly.push_back(mesh.mesh_edges[current_edge].rightPoly);
                    disconnected = false;
                    break;
                }else if(current_point == mesh.mesh_edges[edges[i]].child ) {
                    current_edge = edges[i];
                    current_point = mesh.mesh_edges[current_edge].parent;
                    orderedVerts.push_back(current_point);
                    edges.erase(edges.begin() + i);
                    orderedLeftPoly.push_back(mesh.mesh_edges[current_edge].rightPoly);
                    orderedRightPoly.push_back(mesh.mesh_edges[current_edge].leftPoly);
                    disconnected = false;
                    break;
                }
            }
            if(disconnected){ //disconnected
                if(current_point == edge.parent){
                    current_point = edge.child;
                    orderedVerts.push_back(current_point);
                    orderedLeftPoly.push_back(-2);
                    orderedRightPoly.push_back(-2);
                }
                if(current_point == edge.child){
                    current_point = edge.parent;
                    orderedVerts.push_back(current_point);
                    orderedLeftPoly.push_back(-2);
                    orderedRightPoly.push_back(-2);
                }
            }
        }
        if(current_point==edge.parent){
            p = orderedVerts.size() - 1;
        }
        if(current_point==edge.child){
            c = orderedVerts.size() - 1;
        }
        beforeP = p;
        robustOrientation winding = robustOrientation::kCollinear;

        while(winding == robustOrientation::kCollinear){
            beforeP = beforeP > 0 ? beforeP - 1 : orderedVerts.size()-1;
            winding = Orient(mesh.mesh_vertices[orderedVerts[beforeP]].p,
                             mesh.mesh_vertices[orderedVerts[p]].p,
                             mesh.mesh_vertices[orderedVerts[c]].p);
            if(beforeP == c){
                break;
            }
        }
        if(winding == robustOrientation::kCollinear){
            beforeP = beforeP > 0 ? beforeP - 1 : orderedVerts.size()-1;
            winding = Orient(mesh.mesh_vertices[orderedVerts[beforeP]].p,
                             mesh.mesh_vertices[orderedVerts[c]].p,
                             mesh.mesh_vertices[orderedVerts[p]].p);
            if(beforeP == p){
                std::cout << "Invalid segment!!"<<std::endl;
                return;
            }
        }
        std::vector<SearchNode> left_nodes, right_nodes;
        int S = orderedVerts.size();
        int i, j;
        SearchNode tmp;
        robustOrientation ori;
        tmp.transitionR.isIntersection = false;
        tmp.transitionL.isIntersection = false;
        tmp.predecessor = NULL;
        std::vector<int> rewindedVerts, rewindedLeftPoly, rewindedRightPoly;
        if(winding == robustOrientation::kRightTurn){
            //is rewinding right??
            for(int i=S; i > 0; i--){
                rewindedVerts.push_back(orderedVerts[i % S]);
                rewindedLeftPoly.push_back(orderedRightPoly[(i+1)%S]);
                rewindedRightPoly.push_back(orderedLeftPoly[(i+1)%S]);
                if(orderedVerts[i % S]==edge.parent){
                    p = rewindedVerts.size() - 1;
                }
                if(orderedVerts[i % S]==edge.child){
                    c = rewindedVerts.size() - 1;
                }
            }
            orderedVerts = rewindedVerts;
            orderedRightPoly = rewindedRightPoly;
            orderedLeftPoly = rewindedLeftPoly;
        }
        // right side
        i = p;
        tmp.leftRootVertex = edge.child;
        tmp.rightRootVertex = edge.parent;
        while(i!=c){
            tmp.transitionR.p = orderedVerts[i];
            tmp.transitionL.p = orderedVerts[(i+1) % S];
            tmp.rightVertex = orderedVerts[i];
            tmp.leftVertex = orderedVerts[(i+1) % S];
            tmp.nextPolygon = orderedRightPoly[(i+1) % S];
            if(tmp.nextPolygon == -2){
                i = (i + 1) % S;
                continue;
            }
            tmp.comingFrom = orderedLeftPoly[(i+1) % S];
            tmp.rootR.p = tmp.rightRootVertex;
            tmp.rootR.isIntersection = false;
            tmp.rootL.p = tmp.leftRootVertex;
            tmp.rootL.isIntersection = false;
            // computing left and right root by checking orient with all other init points on that side
            // important! intersections are oriented a->b from root segment
            j = (i + 1) % S;
            while(j!=c){
                if(tmp.transitionR.p == orderedVerts[p])break;
                if(!tmp.rootL.isIntersection){
                    ori = Orient(mesh.mesh_vertices[tmp.transitionR.p].p,
                                 mesh.mesh_vertices[tmp.rootL.p].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kLeftTurn){
                        // root is limited
                        tmp.rootL.isIntersection = true;
                        tmp.rootL.i.b = tmp.transitionR.p;
                        tmp.rootL.i.a = orderedVerts[j];
                        tmp.rootL.i.c = tmp.leftRootVertex;
                        tmp.rootL.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(mesh.mesh_vertices[tmp.rootL.i.b].p,
                                 mesh.mesh_vertices[tmp.rootL.i.a].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kLeftTurn){
                        tmp.rootL.i.a = orderedVerts[j];
                    }

                }
                j = (j + 1) % S;
            }
            j= i;
            while(j!=p){
                if(tmp.transitionL.p == orderedVerts[c])break;
                if(!tmp.rootR.isIntersection){
                    ori = Orient(mesh.mesh_vertices[tmp.transitionL.p].p,
                                 mesh.mesh_vertices[tmp.rootR.p].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kRightTurn){
                        // root is limited
                        tmp.rootR.isIntersection = true;
                        tmp.rootR.i.b = tmp.transitionL.p;
                        tmp.rootR.i.a = orderedVerts[j];
                        tmp.rootR.i.c = tmp.leftRootVertex;
                        tmp.rootR.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(mesh.mesh_vertices[tmp.rootR.i.b].p,
                                 mesh.mesh_vertices[tmp.rootR.i.a].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kRightTurn){
                        tmp.rootR.i.a = orderedVerts[j];
                    }

                }
                j = j > 0 ? j - 1 : S - 1;
            }
            right_nodes.push_back(tmp);
            initRightNodes[rightCount] = tmp;
            rightCount++;
            i = (i + 1) % S;
        }
        // left side
        i = c;
        tmp.leftRootVertex = edge.parent;
        tmp.rightRootVertex = edge.child;
        while(i!=p){
            tmp.transitionR.p = orderedVerts[i];
            tmp.transitionL.p = orderedVerts[(i+1) % S];
            tmp.rightVertex = orderedVerts[i];
            tmp.leftVertex = orderedVerts[(i+1) % S];
            tmp.nextPolygon = orderedRightPoly[(i+1) % S];
            if(tmp.nextPolygon == -2){
                i = (i + 1) % S;
                continue;
            }
            tmp.comingFrom = orderedLeftPoly[(i+1) % S];
            tmp.rootR.p = tmp.rightRootVertex;
            tmp.rootR.isIntersection = false;
            tmp.rootL.p = tmp.leftRootVertex;
            tmp.rootL.isIntersection = false;
            j = (i + 1) % S;
            while(j!=p){
                if(tmp.transitionR.p == orderedVerts[c]) break;
                if(!tmp.rootL.isIntersection){
                    ori = Orient(mesh.mesh_vertices[tmp.transitionR.p].p,
                                 mesh.mesh_vertices[tmp.rootL.p].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kLeftTurn){
                        // root is limited
                        tmp.rootL.isIntersection = true;
                        tmp.rootL.i.b = tmp.transitionR.p;
                        tmp.rootL.i.a = orderedVerts[j];
                        tmp.rootL.i.c = tmp.leftRootVertex;
                        tmp.rootL.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(mesh.mesh_vertices[tmp.rootL.i.b].p,
                                 mesh.mesh_vertices[tmp.rootL.i.a].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kLeftTurn){
                        tmp.rootL.i.a = orderedVerts[j];
                    }

                }
                j = (j + 1) % S;
            }
            j= i;
            while(j!=c){
                if(tmp.transitionL.p == orderedVerts[p]) break;
                if(!tmp.rootR.isIntersection){
                    ori = Orient(mesh.mesh_vertices[tmp.transitionL.p].p,
                                 mesh.mesh_vertices[tmp.rootR.p].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kRightTurn){
                        // root is limited
                        tmp.rootR.isIntersection = true;
                        tmp.rootR.i.b = tmp.transitionL.p;
                        tmp.rootR.i.a = orderedVerts[j];
                        tmp.rootR.i.c = tmp.leftRootVertex;
                        tmp.rootR.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(mesh.mesh_vertices[tmp.rootR.i.b].p,
                                 mesh.mesh_vertices[tmp.rootR.i.a].p,
                                 mesh.mesh_vertices[orderedVerts[j]].p);
                    if(ori==robustOrientation::kRightTurn){
                        tmp.rootR.i.a = orderedVerts[j];
                    }

                }
                j = j > 0 ? j - 1 : S - 1;
            }
            left_nodes.push_back(tmp);
            initLeftNodes[leftCount] = tmp;
            leftCount++;
            i = (i + 1) % S;
        }
        return;
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

    void EdgeVisibility::visualise_vertex_indexes() {
        cgm_drawer.Close();

        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        int i = 0;

        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 20);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
            }
        }

        double x_min, x_max, y_min, y_max;
        geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
        geom::Polygon<double> border = {
                {x_min, y_min},
                {x_min, y_max},
                {x_max, y_max},
                {x_max, y_min},
        };
        cgm_drawer.OpenPDF("vertices.pdf");
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

        for(auto v : mesh.mesh_vertices){
            this->visualise_named_point(v.p, 2, std::to_string(i++));
        }


        this->reset_visu();
    }

    void
    EdgeVisibility::visualise_heatmap(std::vector<Point> &points, std::vector<double> &compT, double tMax, double tMin,
                                      std::string name) {
        cgm_drawer.Close();

        int resolution = 300;
        double width = mesh.max_x - mesh.min_x;
        double height = mesh.max_y - mesh.min_y;
        double widthIncrement = width / resolution;
        double heightIncrement = height / resolution;
        std::vector<std::vector<double>> timeCols;
        std::vector<std::vector<int>> countCols;
        std::vector<double> timeRows;
        std::vector<int> countRows;

        for(int row = 0; row < resolution; row++){
            timeRows.push_back(0.0);
            countRows.push_back(1);
        }
        for(int col = 0; col < resolution; col++){
            timeCols.push_back(timeRows);
            countCols.push_back(countRows);
        }

        geom::Polygons<double> free;
        geom::Points<double> vertices;
        int i = 0;
        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 20);
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
            }
        }

        double x_min, x_max, y_min, y_max;
        geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
        geom::Polygon<double> border = {
                {x_min, y_min},
                {x_min, y_max},
                {x_max, y_max},
                {x_max, y_min},
        };
        cgm_drawer.OpenPDF(name);
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

        for(int i = 0; i < points.size(); i++){
            auto pos = points[i];
            pos.x = pos.x - mesh.min_x;
            pos.y = pos.y - mesh.min_y;
            // auto t = (compT[i] - tMin + 0.00001) / (tMax - tMin);
            auto t = compT[i];
            //std::cout << pos << std::endl;
            timeCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)] = timeCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)] + t;
            countCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)]++;
        }
        double tMax2 = 0;
        double tMin2 = 1;
        for(int col = 0; col < resolution; col++){
            for(int row = 0; row < resolution; row++){
                double heat = ((timeCols[col][row] / countCols[col][row]) - tMin) / (tMax - tMin);
                timeCols[col][row] = heat;
                if(tMax2 < heat) tMax2 = heat;
                if(tMin2 > heat and heat > 0) tMin2 = heat;
            }
        }
        for(int col = 0; col < resolution; col++){
            for(int row = 0; row < resolution; row++){
                Point p = {col * widthIncrement, row*heightIncrement};
                double heat = (timeCols[col][row] - tMin2) / (tMax2 - tMin2);
                this->visualise_heat_point(p, heat, widthIncrement, heightIncrement);
            }
        }

        this->reset_visu();
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
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
            }
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
        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 20);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
            }
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
        cgm_drawer.SaveToPng("debug_visu.png");
    }

    void
    EdgeVisibility::visualise_segment(Point A, Point B, int color, float opacity) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        vertex2.x = B.x - mesh.min_x;
        vertex2.y = B.y - mesh.min_y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_point(Point A, int color){
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_heat_point(Point A, double heat, double heightIncrement, double widthIncrement) {
        geom::Point<double> vertex;
        geom::Polygon<double> polygon;
        if(heat <= 0) return;

        int b = std::max(0, int(255*(1 - 2*heat)));
        int r = std::max(0, int(255*(2*heat - 1)));
        int g = 255 - b -r;

        vertex.x = A.x;
        vertex.y = A.y;
        polygon.push_back(vertex);

        vertex.x = A.x;
        vertex.y = A.y + heightIncrement;
        polygon.push_back(vertex);

        vertex.x = A.x + widthIncrement;
        vertex.y = A.y + heightIncrement;
        polygon.push_back(vertex);

        vertex.x = A.x + widthIncrement;
        vertex.y = A.y;
        polygon.push_back(vertex);



        cgm_drawer.DrawPolygon(polygon, cgm::RGB({r,g,b}), 0.5);

        return;

    }

    void
    EdgeVisibility::visualise_named_point(Point A, int color, std::string str){
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color], 0.5, str);
        return;

    }

    void
    EdgeVisibility::visualise_polygon(std::vector<Point>& p, int color) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorYellow};
        cgm::RGB light_colors[4] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue, cgm::kColorLightYellow};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x - mesh.min_x;
            vertex.y = v.y - mesh.min_y;
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