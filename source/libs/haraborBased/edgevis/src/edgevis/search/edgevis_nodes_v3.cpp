#include "edgevis/search/edge_visibility.h"
namespace edgevis {
    bool
    EdgeVisibility::check_visibility_on2(Point &a, OptimNodeV2 &on, Edge &e, bool right) {
        if(on.root_L.isIntersection){
            return false;
        }
        if(on.root_R.isIntersection){
            return false;
        }
        robustOrientation rightOri = Orient(this->evaluate_intersection(on.P),
                                            this->evaluate_intersection(on.root_R),
                                            a);
        robustOrientation leftOri = Orient(this->evaluate_intersection(on.P),
                                           this->evaluate_intersection(on.root_L),
                                           a);
        if(rightOri != robustOrientation::kLeftTurn && leftOri != robustOrientation::kRightTurn){
            return true;
        }
        return false;
    }

    void
    EdgeVisibility::precompute_edges_optimnodesV2(){
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
        for (Edge &e: mesh.mesh_edges) {
            r_v.clear();
            l_v.clear();
            if(e.leftPoly >= 0){
                leftPoly = mesh.mesh_polygons[e.leftPoly];
                if(leftPoly.vertices.size() != 3){
                    std::cerr <<
                              "[edgevis_nodes_v2.cpp :: precompute_edges_optimnodesV2] Can be used only for triangular mesh!"
                              << std::endl;
                    return;
                }
                for(int i = 0; i < 3; i++){
                    if(leftPoly.vertices[i] != e.parent and leftPoly.vertices[i] != e. child){
                        leftVertex = mesh.mesh_vertices[leftPoly.vertices[i]].p;
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
                rightPoly = mesh.mesh_polygons[e.rightPoly];
                if(rightPoly.vertices.size() != 3){
                    std::cerr <<
                              "[edgevis_nodes_v2.cpp :: precompute_edges_optimnodesV2] Can be used only for triangular mesh!"
                              << std::endl;
                    return;
                }
                for(int i = 0; i < 3; i++){
                    if(rightPoly.vertices[i] != e.parent and rightPoly.vertices[i] != e. child){
                        rightVertex = mesh.mesh_vertices[rightPoly.vertices[i]].p;
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

    std::vector<Point>
    EdgeVisibility::find_point_visibility_optim2(Point p, bool debug, double &steps) {
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
        std::vector<OptimNodeV2> *vec;
        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->leftOptimNodesV2.size() > 0) {
                vec = &e->leftOptimNodesV2;
            }else if((pl.poly1 == e->leftPoly && e->rightOptimNodesV2.size() > 0)) {
                vec = &e->rightOptimNodesV2;
            }
            for(auto node : *vec){
                if(node.isAlwaysVisible){
                    current_state = STATE::VISIBLE;
                }else{
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

}