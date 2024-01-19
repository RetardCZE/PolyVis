#include "edgevis/structs/mesh.h"

namespace edgevis{
    void
    Mesh::precompute_edges_searchnodes() {
        std::vector<SearchNode> r_v;
        std::vector<SearchNode> l_v;
        int edge = 0;
        // if(max_poly_sides != 3) {std::cerr << "Need triangular mesh!" << std::endl; return;}
        for (Edge& e : this->mesh_edges){
            if(e.leftPoly == -1 or e.rightPoly == -1){
                edge++;
                continue;
            }
            r_v.clear(); l_v.clear();
            //std::cout << "someshit right" << std::endl;
            r_v = this->find_edge_visibility(edge, true, false);
            //std::cout << "someshit left" << std::endl;
            l_v = this->find_edge_visibility(edge, false, false);
            e.right_nodes.reserve(r_v.size());
            e.left_nodes.reserve(l_v.size());
            e.right_nodes.insert(e.right_nodes.end(), r_v.begin(), r_v.end());
            e.left_nodes.insert(e.left_nodes.end(), l_v.begin(), l_v.end());
            edge++;
        }
        searchnodes_precomputed = true;
    }

    void
    Mesh::precompute_edges_optimnodesV1() {
        if (!searchnodes_precomputed) {
            this->precompute_edges_searchnodes();
        }
        std::vector <OptimNode> r_v;
        std::vector <OptimNode> l_v;
        OptimNode on1, on2;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: this->mesh_edges) {
            r_v = compute_side_optimnodesV1(e, true);
            l_v = compute_side_optimnodesV1(e, false);
            e.rightOptNodes.reserve(r_v.size());
            e.leftOptNodes.reserve(l_v.size());
            e.rightOptNodes.insert(e.rightOptNodes.end(), r_v.begin(), r_v.end());
            e.leftOptNodes.insert(e.leftOptNodes.end(), l_v.begin(), l_v.end());
            edge++;
        }
        optimnodes1_precomputed = true;
    }

    void
    Mesh::precompute_edges_optimnodesV2(){
        if(!optimnodes1_precomputed){
            this->precompute_edges_optimnodesV1();
        }
        Polygon leftPoly, rightPoly;
        Point leftVertex, rightVertex;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: this->mesh_edges) {
            if(e.leftPoly >= 0){
                leftPoly = this->mesh_polygons[e.leftPoly];
                if(leftPoly.vertices.size() != 3){
                    std::cerr <<
                              "[edgevis_nodes_v2.cpp :: precompute_edges_optimnodesV2] Can be used only for triangular mesh!"
                              << std::endl;
                    return;
                }
                for(int i = 0; i < 3; i++){
                    if(leftPoly.vertices[i] != e.parent and leftPoly.vertices[i] != e. child){
                        leftVertex = this->mesh_vertices[leftPoly.vertices[i]].p;
                    }
                }
                if(e.rightOptNodes.size() > 0) {
                    for (auto &on1: e.rightOptNodes) {
                        on1.isAlwaysVisible = this->check_visibility_on2(leftVertex, on1, e, true);
                    }
                }
            }
            if(e.rightPoly >= 0){
                rightPoly = this->mesh_polygons[e.rightPoly];
                if(rightPoly.vertices.size() != 3){
                    std::cerr <<
                              "[edgevis_nodes_v2.cpp :: precompute_edges_optimnodesV2] Can be used only for triangular mesh!"
                              << std::endl;
                    return;
                }
                for(int i = 0; i < 3; i++){
                    if(rightPoly.vertices[i] != e.parent and rightPoly.vertices[i] != e. child){
                        rightVertex = this->mesh_vertices[rightPoly.vertices[i]].p;
                    }
                }
                if(e.leftOptNodes.size() > 0){
                    for(auto &on1 : e.leftOptNodes){
                        on1.isAlwaysVisible = this->check_visibility_on2(rightVertex, on1, e, false);
                    }
                }
            }
            edge++;
        }
        optimnodes2_precomputed = true;
        return;
    }

    void
    Mesh::precompute_edges_optimnodesV3(){
        if(!optimnodes1_precomputed){
            this->precompute_edges_optimnodesV1();
        }

        for (Edge &e: this->mesh_edges) {
            this->compute_side_optimnodesV3(e, true);
            this->compute_side_optimnodesV3(e, false);
        }
        optimnodes3_precomputed = true;
        return;
    }

    std::vector <OptimNode>
    Mesh::compute_side_optimnodesV1(Edge &edge, bool right) {
        std::vector <OptimNode> v;
        OptimNode on1, on2;

        on1.P.isIntersection = false;
        if (right) {
            on1.P.p = edge.parent;
            on1.root_R.p = edge.parent;
            on1.root_L.p = edge.child;
        } else {
            on1.P.p = edge.child;
            on1.root_R.p = edge.child;
            on1.root_L.p = edge.parent;
        }
        on1.root_R.isIntersection = false;
        on1.root_L.isIntersection = false;
        on1.OT1 = true;
        v.push_back(on1);
        std::vector <SearchNode> *vec = right ? &edge.right_nodes : &edge.left_nodes;
        if (vec->size() > 0) {
            for (auto sn: *vec) {
                this->compute_optimnodesv1(sn, on1, on2);
                on1.OT1 = true;
                on2.OT1 = true;
                if (!(on1.P == v.back().P)) {
                    v.push_back(on1);
                }
                if (!(on2.P == v.back().P)) {
                    v.push_back(on2);
                }
            }
        }

        on1.P.isIntersection = false;
        if (right) {
            on1.P.p = edge.child;
            on1.root_R.p = edge.parent;
            on1.root_L.p = edge.child;
        } else {
            on1.P.p = edge.parent;
            on1.root_R.p = edge.child;
            on1.root_L.p = edge.parent;
        }
        on1.root_R.isIntersection = false;
        on1.root_L.isIntersection = false;
        on1.OT1 = true;
        v.push_back(on1);
        return v;
    }

    void
    Mesh::compute_optimnodesv1(SearchNode &node, OptimNode &o1, OptimNode &o2) {
        SearchNode *parent;
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
            if (o1.P.isIntersection) {
                o1.root_R = node.rootL;
            } else if (!parent->transitionR.isIntersection) {
                int root_dir = o1.root_R.isIntersection ? o1.root_R.i.a : o1.root_R.p;
                if (Orient(this->mesh_vertices[o1.P.p].p,
                           this->mesh_vertices[root_dir].p,
                           this->mesh_vertices[parent->transitionR.p].p, useRobustOrientatation)
                    ==
                    robustOrientation::kRightTurn) {
                    o1.root_R.isIntersection = true;
                    o1.root_R.i.b = o1.P.p;
                    o1.root_R.i.a = parent->transitionR.p;
                    o1.root_R.i.c = node.rightRootVertex;
                    o1.root_R.i.d = node.leftRootVertex;
                }
            }

            if (o2.P.isIntersection) {
                o2.root_L = node.rootR;
            } else if (!parent->transitionL.isIntersection) {
                int root_dir = o2.root_L.isIntersection ? o2.root_L.i.a : o2.root_L.p;
                if (Orient(this->mesh_vertices[o2.P.p].p,
                           this->mesh_vertices[root_dir].p,
                           this->mesh_vertices[parent->transitionL.p].p, useRobustOrientatation)
                    ==
                    robustOrientation::kLeftTurn) {
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

    bool
    Mesh::check_visibility_on2(Point &a, OptimNode &on, Edge &e, bool right) {
        on.OT2 = true;
        if(this->evaluate_intersection(on.root_L).distance(this->mesh_vertices[e.parent].p) > 1e-3 ){
            return false;
        }
        if(this->evaluate_intersection(on.root_R).distance(this->mesh_vertices[e.child].p) > 1e-3 ){
            return false;
        }
        robustOrientation rightOri = Orient(this->evaluate_intersection(on.P),
                                            this->evaluate_intersection(on.root_R),
                                            a, useRobustOrientatation);
        robustOrientation leftOri = Orient(this->evaluate_intersection(on.P),
                                           this->evaluate_intersection(on.root_L),
                                           a, useRobustOrientatation);
        if(rightOri != robustOrientation::kLeftTurn && leftOri != robustOrientation::kRightTurn){
            return true;
        }
        return false;
    }


    void
    Mesh::compute_side_optimnodesV3(Edge &edge, bool right){
        std::vector<OptimNode> *vec = right ? &edge.rightOptNodes : &edge.leftOptNodes;

        int l,r;
        if(right){
            l = edge.child;
            r = edge.parent;
        }else{
            l = edge.parent;
            r = edge.child;
        }
        Point right2left = this->mesh_vertices[l].p - this->mesh_vertices[r].p;
        Point left2right = this->mesh_vertices[r].p - this->mesh_vertices[l].p;
        edge.INITED3 = true;

        for(auto &on : *vec){
            on.TESTER = true;
            if(!on.root_L.isIntersection){
                if(on.root_L.p != r) on.root_L_order = 0;
                if(on.root_L.p == r) on.root_L_order = 1;
            }else{
                Point vector = this->evaluate_intersection(on.root_L) - this->mesh_vertices[l].p;
                if(abs(vector.x) > 1e-8){
                    on.root_L_order = vector.x / left2right.x;
                }else if(abs(vector.y) > 1e-8){
                    on.root_L_order = vector.y / left2right.y;
                }else{
                    on.root_L_order = 0;
                }
                if(on.root_L_order < 1e-16) on.root_L_order = 0;
            }
            if(!on.root_R.isIntersection){
                if(on.root_R.p != l) on.root_R_order = 0;
                if(on.root_R.p == l) on.root_R_order = 1;
            }else{
                Point vector = this->evaluate_intersection(on.root_R) - this->mesh_vertices[r].p;
                if(abs(vector.x) > 1e-8){
                    on.root_R_order = vector.x / right2left.x;
                }else if(abs(vector.y) > 1e-8){
                    on.root_R_order = vector.y / right2left.y;
                }else{
                    on.root_R_order = 0;
                }
                if(on.root_R_order < 1e-16) on.root_R_order = 0;
            }
        }
        return;
    }

}