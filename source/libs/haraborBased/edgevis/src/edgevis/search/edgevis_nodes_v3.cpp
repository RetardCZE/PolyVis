#include "edgevis/search/edge_visibility.h"
namespace edgevis {
    bool
    EdgeVisibility::optimnodeV3_is_further(OptimNodeV3 &base, OptimNodeV3 &compared, bool right){
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
        robustOrientation orientation = Orient(mesh.mesh_vertices[root.i.b].p,
                                               mesh.mesh_vertices[root.i.a].p,
                                               basePoint);
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

    void
    EdgeVisibility::optimnodesV3_compute_roots_positions(std::vector<OptimNodeV3> &list) {
        if(list.size() == 0){
            return;
        }
        std::vector<int> orderLeft, orderRight;
        orderLeft.push_back(0);
        orderRight.push_back(0);
        auto current_push = orderLeft.end();

        for(int i = 1; i < list.size(); i++){ // compare with all except itself
            current_push = orderLeft.end();
            for(int j = 0; j < orderLeft.size(); j++){
                if(!optimnodeV3_is_further(list[orderLeft[j]], list[i], false)) {
                    current_push = orderLeft.begin()+j;
                    break;
                }
            }
            orderLeft.insert(current_push, i);

            current_push = orderRight.end();
            for(int j = 0; j < orderRight.size(); j++){
                if(!optimnodeV3_is_further(list[orderRight[j]], list[i], true)) {
                    current_push = orderRight.begin()+j;
                    break;
                }
            }
            orderRight.insert(current_push, i);
        }

        for(int i = 0; i < list.size(); i++){
            list[orderLeft[i]].root_L_order = i;
            list[orderRight[i]].root_R_order = i;
        }
        return;
    }

    std::vector<OptimNodeV3>
    EdgeVisibility::compute_side_optimnodesV3(Edge &edge, bool right){
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
        Point right2left = mesh.mesh_vertices[l].p - mesh.mesh_vertices[r].p;
        Point left2right = mesh.mesh_vertices[r].p - mesh.mesh_vertices[l].p;

        for(auto on1 : side){
            on3.root_L = on1.root_L;
            on3.root_R = on1.root_R;
            on3.P = on1.P;
            if(!on3.root_L.isIntersection){
                if(on3.root_L.p != r) on3.root_L_order = 0;
                if(on3.root_L.p == r) on3.root_L_order = 1;
            }else{
                Point vector = this->evaluate_intersection(on3.root_L) - mesh.mesh_vertices[l].p;
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
                Point vector = this->evaluate_intersection(on3.root_R) - mesh.mesh_vertices[r].p;
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

    void
    EdgeVisibility::precompute_edges_optimnodesV3(){
        if(!optimnodes1_precomputed){
            this->precompute_edges_optimnodesV1();
        }
        std::vector<OptimNodeV3*> rList, lList;
        std::vector<OptimNodeV3> r_v;
        std::vector<OptimNodeV3> l_v;


        OptimNodeV3 on3;
        SearchNode sn;
        int edge = 0;
        for (Edge &e: mesh.mesh_edges) {
            r_v.clear();
            r_v = this->compute_side_optimnodesV3(e, true);
            l_v.clear();
            l_v = this->compute_side_optimnodesV3(e, false);
            /*

            if(r_v.size() > 0){
                this->reset_visu();
                std::vector<Point> ps;
                for(auto p : r_v){
                    Point show = this->evaluate_intersection(p.P);
                    std::cout << show << p.root_L_order << std::endl;
                    ps.push_back(show);
                    if(p.root_L.isIntersection){
                        this->visualise_named_point(show, 0, std::to_string(p.root_L_order));
                    }else{
                        this->visualise_named_point(show, 0, std::to_string(0));
                    }

                }
                this->visualise_polygon(ps, 1, true);
                getchar();
            }

             */
            e.rightOptimNodesV3.reserve(r_v.size());
            e.leftOptimNodesV3.reserve(l_v.size());
            e.rightOptimNodesV3.insert(e.rightOptimNodesV3.end(), r_v.begin(), r_v.end());
            e.leftOptimNodesV3.insert(e.leftOptimNodesV3.end(), l_v.begin(), l_v.end());
            edge++;
        }
        optimnodes3_precomputed = true;
        return;
    }

    std::vector<Point>
    EdgeVisibility::find_point_visibility_optim3(Point p, bool debug, double &steps, int &debugEdge) {
        if(!optimnodes3_precomputed)
            this->precompute_edges_optimnodesV3();

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
        std::vector<OptimNodeV3> *vec;
        std::vector<OptimNodeV3> debugger;
        OptimNodeV3 node, last_node;
        int lastRight, lastLeft;
        int l,r;
        Point right2left;
        Point left2right;
        float currentLimitation;

        if(debug)
            debugEdge = debugEdge % edges.size();
        int counter = 0;
        bool skip = false;
        for (Edge* e :edges) {
            if(debug && counter!=debugEdge){
                counter++;
                continue;
            }
            counter++;

            if (pl.poly1 == e->rightPoly && e->leftOptimNodesV1.size() > 0) {
                vec = &e->leftOptimNodesV3;
                l = e->parent;
                r = e->child;
            }else if((pl.poly1 == e->leftPoly && e->rightOptimNodesV1.size() > 0)) {
                vec = &e->rightOptimNodesV3;
                l = e->child;
                r = e->parent;
            }
            right2left = mesh.mesh_vertices[l].p - mesh.mesh_vertices[r].p;
            left2right = mesh.mesh_vertices[r].p - mesh.mesh_vertices[l].p;
            auto iterator = vec->begin();
            while(iterator != vec->end()){
                node = *iterator;
                if(false and iterator != vec->begin()){
                    int color;
                    switch(last_state){
                        case STATE::VISIBLE:
                            color = 1;
                            break;
                        case STATE::LEFT:
                            color = 2;
                            break;
                        case STATE::RIGHT:
                            color = 0;
                            break;
                        default:
                            break;
                    }
                    this->visualise_point(this->evaluate_intersection(last_node.P), color, true);
                    std::cout << node.root_L_order << " | " << node.root_R_order << std::endl;
                    getchar();
                }

                steps++;
                if (node.root_L.isIntersection) {
                    left_parent = node.root_L.i.a;
                    left_child = node.P.isIntersection ? node.root_L.i.b : node.P.p;
                } else {
                    left_parent = node.root_L.p;
                    left_child = node.P.isIntersection ? node.P.i.b : node.P.p;
                }
                if (node.root_R.isIntersection) {
                    right_parent = node.root_R.i.a;
                    right_child = node.P.isIntersection ? node.root_R.i.b : node.P.p;
                } else {
                    right_parent = node.root_R.p;
                    right_child = node.P.isIntersection ? node.P.i.b : node.P.p;
                }
                robustOrientation rightOri = Orient(mesh.mesh_vertices[right_child].p,
                                                    mesh.mesh_vertices[right_parent].p,
                                                    p);
                robustOrientation leftOri = Orient(mesh.mesh_vertices[left_child].p,
                                                   mesh.mesh_vertices[left_parent].p,
                                                   p);
                if (rightOri != robustOrientation::kLeftTurn && leftOri != robustOrientation::kRightTurn) {
                    current_state = STATE::VISIBLE;
                } else {
                    if (rightOri == robustOrientation::kLeftTurn) {
                        current_state = STATE::RIGHT;
                    } else {
                        current_state = STATE::LEFT;
                    }
                }

                if(current_state != last_state){
                    if(last_state == STATE::RIGHT){
                        // leaving RIGHT causes triggering of intersection
                        LineLineIntersectionNotCollinear(p, last_visible, last_point, this->evaluate_intersection(node.P), I);
                        out.push_back(I);
                    }
                    if(current_state == STATE::RIGHT and last_state == STATE::VISIBLE){
                        Point n = this->evaluate_intersection(last_node.P);
                        Point i;
                        LineLineIntersectionNotCollinear(p,
                                                         n,
                                                         mesh.mesh_vertices[e->parent].p,
                                                         mesh.mesh_vertices[e->child].p,
                                                         i);
                        Point vector = i - mesh.mesh_vertices[r].p;
                        if(vector.x != 0){
                            currentLimitation = abs(vector.x / right2left.x);
                        }else if(vector.y != 0){
                            currentLimitation = abs(vector.y / right2left.y);
                        }else{
                            currentLimitation = 0;
                        }
                        while(currentLimitation < iterator->root_R_order){
                            if(false){
                                this->visualise_point(this->evaluate_intersection(iterator->P), 0, true);
                            getchar();}
                            iterator++;
                        }
                        iterator--;
                    }
                    if(current_state == STATE::LEFT){
                        // entering left causes hanging intersection
                        segment_holder[0] = last_point;
                        segment_holder[1] = this->evaluate_intersection(node.P);
                        if(last_state == STATE::VISIBLE){
                            Point n = this->evaluate_intersection(last_node.P);
                            Point i;
                            LineLineIntersectionNotCollinear(p,
                                                             n,
                                                             mesh.mesh_vertices[e->parent].p,
                                                             mesh.mesh_vertices[e->child].p,
                                                             i);
                            Point vector = i - mesh.mesh_vertices[l].p;
                            if(vector.x != 0){
                                currentLimitation = abs(vector.x / left2right.x);
                            }else if(vector.y != 0){
                                currentLimitation = abs(vector.y / left2right.y);
                            }else{
                                currentLimitation = 0;
                            }
                            while(currentLimitation < iterator->root_L_order){
                                if(false){
                                    this->visualise_point(this->evaluate_intersection(iterator->P), 2, true);
                                getchar();}
                                iterator++;
                            }
                            iterator--;
                        }
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
                last_node = node;
                last_state = current_state;
                iterator++;
            }

            if(debug){
                if(debugger.size() > 0){
                    for(auto p:debugger){
                        this->visualise_point(this->evaluate_intersection(p.P), 0, false);
                    }
                }
                break;
            }
        }

        return out;

    }


}