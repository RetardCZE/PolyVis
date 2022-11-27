#include "edgevis/search/edge_visibility.h"
namespace edgevis {

    std::vector<Point>
    EdgeVisibility::find_point_visibility_optim1(Point p, bool debug, double &steps, int &debugEdge) {
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
        std::vector<Point> debugger;
        if(debug)
            debugEdge = debugEdge % edges.size();
        int counter = 0;
        for (Edge* e :edges) {
            if(debug && counter!=debugEdge){
                counter++;
                continue;
            }
            counter++;

            if (pl.poly1 == e->rightPoly && e->leftOptimNodesV1.size() > 0) {
                vec = &e->leftOptimNodesV1;
            }else if((pl.poly1 == e->leftPoly && e->rightOptimNodesV1.size() > 0)) {
                vec = &e->rightOptimNodesV1;
            }
            for(auto node : *vec){
                if(debug){
                    debugger.push_back(this->evaluate_intersection(node.P));
                }
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

            if(debug){
                this->reset_visu();
                if(debugger.size() > 0)
                    this->visualise_polygon(debugger, 2, false);
                break;
            }
        }

        return out;
    }

    std::vector <OptimNodeV1>
    EdgeVisibility::compute_side_optimnodesV1(Edge &edge, bool right) {
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

    void EdgeVisibility::precompute_edges_optimnodesV1() {
        if (!searchnodes_precomputed) {
            this->precompute_edges_searchnodes();
        }
        std::vector <OptimNodeV1> r_v;
        std::vector <OptimNodeV1> l_v;
        OptimNodeV1 on1, on2;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: mesh.mesh_edges) {
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
    EdgeVisibility::compute_optimnodesv1(SearchNode &node, OptimNodeV1 &o1, OptimNodeV1 &o2) {
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
                if (Orient(mesh.mesh_vertices[o1.P.p].p,
                           mesh.mesh_vertices[root_dir].p,
                           mesh.mesh_vertices[parent->transitionR.p].p)
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
                if (Orient(mesh.mesh_vertices[o2.P.p].p,
                           mesh.mesh_vertices[root_dir].p,
                           mesh.mesh_vertices[parent->transitionL.p].p)
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
}