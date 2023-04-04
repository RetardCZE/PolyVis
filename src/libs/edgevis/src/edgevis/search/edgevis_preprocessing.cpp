#include "edgevis/structs/mesh.h"

namespace edgevis{
    void
    Mesh::precompute_edges_searchnodes() {
        std::vector<SearchNode> r_v;
        std::vector<SearchNode> l_v;
        int edge = 0;
        for (Edge& e : this->mesh_edges){
            if(e.leftPoly == -1 or e.rightPoly == -1){
                edge++;
                continue;
            }
            r_v.clear(); l_v.clear();
            r_v = this->find_edge_visibility(edge, true, false);
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
        std::vector <OptimNodeV1> r_v;
        std::vector <OptimNodeV1> l_v;
        OptimNodeV1 on1, on2;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: this->mesh_edges) {
            r_v = compute_side_optimnodesV1(e, true);
            l_v = compute_side_optimnodesV1(e, false);
            e.rightOptimNodesV1.reserve(r_v.size());
            e.leftOptimNodesV1.reserve(l_v.size());
            e.rightOptimNodesV1.insert(e.rightOptimNodesV1.end(), r_v.begin(), r_v.end());
            e.leftOptimNodesV1.insert(e.leftOptimNodesV1.end(), l_v.begin(), l_v.end());
            edge++;
        }
        optimnodes1_precomputed = true;
    }

    void
    Mesh::precompute_edges_optimnodesV2(){
        if(!optimnodes1_precomputed){
            this->precompute_edges_optimnodesV1();
        }
        std::vector<OptimNodeV2> r_v;
        std::vector<OptimNodeV2> l_v;
        Polygon leftPoly, rightPoly;
        Point leftVertex, rightVertex;
        OptimNodeV2 on2;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: this->mesh_edges) {
            r_v.clear();
            l_v.clear();
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
                if(e.rightOptimNodesV1.size() > 0) {
                    for (auto &on1: e.rightOptimNodesV1) {
                        on2.root_L = on1.root_L;
                        on2.root_R = on1.root_R;
                        on2.P = on1.P;
                        on2.isAlwaysVisible = this->check_visibility_on2(leftVertex, on2, e,
                                                                         true);

                        r_v.push_back(on2);
                    }
                    e.rightOptimNodesV2.reserve(r_v.size());
                    e.rightOptimNodesV2.insert(e.rightOptimNodesV2.end(), r_v.begin(), r_v.end());
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
                if(e.leftOptimNodesV1.size() > 0){
                    for(auto &on1 : e.leftOptimNodesV1){
                        on2.root_L = on1.root_L;
                        on2.root_R = on1.root_R;
                        on2.P = on1.P;
                        on2.isAlwaysVisible = this->check_visibility_on2(rightVertex, on2, e,
                                                                         false);

                        l_v.push_back(on2);
                    }
                    e.leftOptimNodesV2.reserve(l_v.size());
                    e.leftOptimNodesV2.insert(e.leftOptimNodesV2.end(), l_v.begin(), l_v.end());
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
        std::vector<OptimNodeV3*> rList, lList;
        std::vector<OptimNodeV3> r_v;
        std::vector<OptimNodeV3> l_v;


        OptimNodeV3 on3;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: this->mesh_edges) {
            r_v.clear();
            r_v = this->compute_side_optimnodesV3(e, true);
            l_v.clear();
            l_v = this->compute_side_optimnodesV3(e, false);

            e.rightOptimNodesV3.reserve(r_v.size());
            e.leftOptimNodesV3.reserve(l_v.size());
            e.rightOptimNodesV3.insert(e.rightOptimNodesV3.end(), r_v.begin(), r_v.end());
            e.leftOptimNodesV3.insert(e.leftOptimNodesV3.end(), l_v.begin(), l_v.end());
            edge++;
        }
        optimnodes3_precomputed = true;
        return;
    }

    std::vector <OptimNodeV1>
    Mesh::compute_side_optimnodesV1(Edge &edge, bool right) {
        std::vector <OptimNodeV1> v;
        OptimNodeV1 on1, on2;

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
        v.push_back(on1);
        std::vector <SearchNode> *vec = right ? &edge.right_nodes : &edge.left_nodes;
        if (vec->size() > 0) {
            for (auto sn: *vec) {
                this->compute_optimnodesv1(sn, on1, on2);
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
        v.push_back(on1);
        return v;
    }

    void
    Mesh::compute_optimnodesv1(SearchNode &node, OptimNodeV1 &o1, OptimNodeV1 &o2) {
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
    Mesh::check_visibility_on2(Point &a, OptimNodeV2 &on, Edge &e, bool right) {
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

    bool
    Mesh::optimnodeV3_is_further(OptimNodeV3 &base, OptimNodeV3 &compared, bool right){
        SearchPoint root, baseRoot;
        if(right){
            baseRoot = base.root_R;
            root = compared.root_R;
        }else{
            baseRoot = base.root_L;
            root = compared.root_L;
        }

        /*
         * If root is not defined as an intersection it means it wasn't limited by obstacles and thus is on
         * beginning from given side.
         */
        if(!root.isIntersection)
            return false;

        // Similar as upper condition, but now we know that root is intersection as has to be further than beginning.
        if(!baseRoot.isIntersection)
            return true;
        /*
         * else we need to compute their respective orientation !! side is relevant
         * we get 2 intersections so there's no need in further checks
         * also all root intersections has common target segment c and d
         * respective visibility can be read only from a and b definitions of intersection
         */

        //TODO: Not sure if orientation can be defined in abstract intersection mode so 1 will be evaluated
        Point basePoint = this->evaluate_intersection(baseRoot);
        robustOrientation orientation = Orient(this->mesh_vertices[root.i.b].p,
                                               this->mesh_vertices[root.i.a].p,
                                               basePoint, useRobustOrientatation);
        if(right){
            if(orientation==robustOrientation::kRightTurn){
                return false;
            }
            return true;
        }else{
            if(orientation==robustOrientation::kLeftTurn){
                return false;
            }
            return true;
        }
    }

    std::vector<OptimNodeV3>
    Mesh::compute_side_optimnodesV3(Edge &edge, bool right){
        std::vector<OptimNodeV3> nodes;
        OptimNodeV3 on3;
        std::vector<OptimNodeV1> side;
        if(right){
            side = edge.rightOptimNodesV1;
        }else{
            side = edge.leftOptimNodesV1;
        }
        if(side.size() == 0){
            nodes.clear();
            return nodes;
        }

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

        for(auto on1 : side){
            on3.root_L = on1.root_L;
            on3.root_R = on1.root_R;
            on3.P = on1.P;
            if(!on3.root_L.isIntersection){
                if(on3.root_L.p != r) on3.root_L_order = 0;
                if(on3.root_L.p == r) on3.root_L_order = 1;
            }else{
                Point vector = this->evaluate_intersection(on3.root_L) - this->mesh_vertices[l].p;
                if(vector.x != 0){
                    on3.root_L_order = vector.x / left2right.x;
                }else if(vector.y != 0){
                    on3.root_L_order = vector.y / left2right.y;
                }else{
                    on3.root_L_order = 0;
                }
            }
            if(!on3.root_R.isIntersection){
                if(on3.root_R.p != l) on3.root_R_order = 0;
                if(on3.root_R.p == l) on3.root_R_order = 1;
            }else{
                Point vector = this->evaluate_intersection(on3.root_R) - this->mesh_vertices[r].p;
                if(vector.x != 0){
                    on3.root_R_order = vector.x / right2left.x;
                }else if(vector.y != 0){
                    on3.root_R_order = vector.y / right2left.y;
                }else{
                    on3.root_R_order = 0;
                }
            }
            nodes.push_back(on3);
        }

        return nodes;
    }

}