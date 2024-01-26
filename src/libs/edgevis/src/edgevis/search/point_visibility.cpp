#include "edgevis/structs/mesh.h"
#include <stdlib.h>

namespace edgevis {
    std::vector<Edge*>
    Mesh::get_init_edges(PointLocation pl) {
        /**
         * @brief Return edges of polygon containing given point.
         *
         * Initial edges are all edges that are visible from query point without looking over obstacle or another edge.
         * Currently most common position inside one polygon is implemeted, but on edge or mesh vertex should be added.
         *
         * @param pl 2D point in PointLocation structure - it has relation to underlying mesh.
         * @return Vector of edges directly visible from the point (without looking over another edge).
         */
        std::vector<Edge*> edges;
        switch(pl.type){
            case PointLocation::NOT_ON_MESH:
                edges.clear();
                return edges;
            case PointLocation::IN_POLYGON:
                for( auto e : this->mesh_polygons[pl.poly1].edges ){
                    edges.push_back(&this->mesh_edges[e]);
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

    int
    Mesh::get_point_init_nodes(Point root, edgevis::SearchNode *initNodes, double &T1) {
        /**
         * @brief Return initial search nodes for given point in 2D space.
         *
         * First gets all initial edges that are fully visible from given point.
         * From every edge a search node is created so that search nodes are sorted in
         * CCW order (arbitrary origin) and oriented in the same direction (also CCW).
         *
         * @param root 2D point in the map.
         * @param initNodes pointer to prepared container for initial search nodes.
         * @return Number if initial edges.
         */
        // side - true = transitionR, false = transitionL
        clock.Restart();
        PointLocation pl = this->get_point_location(root);
        T1 = clock.TimeInSeconds();
        clock.Restart();
        int num = 0;
        free_points.clear();
        free_points.push_back(root);


        if(pl.type != PointLocation::Type::IN_POLYGON){
            //std::cout << "WARNING POINT not computable " << std::endl;
            return -1;
        }
        std::vector<Edge*> edges = this->get_init_edges(pl);
        //Polygon& poly = this->mesh_polygons[pl.poly1];
        SearchNode temp;
        temp.predecessor = NULL;
        temp.rootL.isIntersection = false;
        temp.rootR.isIntersection = false;
        temp.rootR.p = -1;
        temp.rootL.p = -1;
        temp.rightRootVertex = -1;
        temp.leftRootVertex = -1;
        temp.transitionR.isIntersection = false;
        temp.transitionL.isIntersection = false;
        temp.comingFrom = pl.poly1;
        for(auto &e : edges){
            if(e->leftPoly == pl.poly1){
                temp.transitionR.p = e->parent;
                temp.transitionL.p = e->child;
                temp.leftVertex = e->child;
                temp.rightVertex = e->parent;
                temp.nextPolygon = e->rightPoly;
            }else{
                temp.transitionR.p = e->child;
                temp.transitionL.p = e->parent;
                temp.leftVertex = e->parent;
                temp.rightVertex = e->child;
                temp.nextPolygon = e->leftPoly;
            }
            initNodes[num++] = temp;
        }
        return num;
    }

    std::vector<Point>
    Mesh::find_point_visibility_optim1(Point p, double &T1, double &T2, double &T3) {
        /**
         * @brief short desription.
         *
         * long description
         *
         * @param param1 param description.
         * @return return value description.
         */

        std::vector<Point> out;
        clock.Restart();
        PointLocation pl = this->get_point_location(p);
        if(pl.type != PointLocation::Type::IN_POLYGON){
            //std::cout << "WARNING POINT not computable " << std::endl;
            T1 = clock.TimeInSeconds();
            T2 = clock.TimeInSeconds();
            return out;
        }
        T1 = clock.TimeInSeconds();
        clock.Restart();
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
        std::vector<OptimNode> *vec;

        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->leftOptNodes.size() > 0) {
                vec = &e->leftOptNodes;
            }else if((pl.poly1 == e->leftPoly && e->rightOptNodes.size() > 0)) {
                vec = &e->rightOptNodes;
            }
            for(auto node : *vec){

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
                robustOrientation rightOri = Orient(this->mesh_vertices[right_child].p,
                                                    this->mesh_vertices[right_parent].p,
                                                    p, useRobustOrientatation);
                robustOrientation leftOri = Orient(this->mesh_vertices[left_child].p,
                                                   this->mesh_vertices[left_parent].p,
                                                   p, useRobustOrientatation);
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
        T2 = clock.TimeInSeconds();
        clock.Restart();
        T3 = -1;
        return out;
    }

    std::vector<Point>
    Mesh::find_point_visibility_optim2(Point p) {
        /**
         * @brief short desription.
         *
         * long description
         *
         * @param param1 param description.
         * @return return value description.
         */

        std::vector<Point> out;
        PointLocation pl = this->get_point_location(p);
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
        std::vector<OptimNode> *vec;


        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->leftOptNodes.size() > 0) {
                vec = &e->leftOptNodes;
            }else if((pl.poly1 == e->leftPoly && e->rightOptNodes.size() > 0)) {
                vec = &e->rightOptNodes;
            }
            for(auto node : *vec){
                if(node.isAlwaysVisible){
                    current_state = STATE::VISIBLE;
                }else{
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
                    robustOrientation rightOri = Orient(this->mesh_vertices[right_child].p,
                                                        this->mesh_vertices[right_parent].p,
                                                        p, useRobustOrientatation);
                    robustOrientation leftOri = Orient(this->mesh_vertices[left_child].p,
                                                       this->mesh_vertices[left_parent].p,
                                                       p, useRobustOrientatation);
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

    std::vector<Point>
    Mesh::find_point_visibility_optim3(Point p) {
        /**
         * @brief short desription.
         *
         * long description
         *
         * @param param1 param description.
         * @return return value description.
         */
        if(!optimnodes3_precomputed) this->precompute_edges_optimnodesV3();

        std::vector<Point> out;
        PointLocation pl = this->get_point_location(p);
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
        std::vector<OptimNode> *vec;
        std::vector<OptimNode> debugger;
        OptimNode node, last_node;

        int lastRight, lastLeft;
        int l,r;
        Point right2left;
        Point left2right;
        float currentLimitation;


        bool skip = false;
        for (Edge* e :edges) {
            //std::cout << "---------------------------------"<<std::endl;
            if (pl.poly1 == e->rightPoly && e->leftOptNodes.size() > 0) {
                vec = &e->leftOptNodes;
                l = e->parent;
                r = e->child;
            }else if((pl.poly1 == e->leftPoly && e->rightOptNodes.size() > 0)) {
                vec = &e->rightOptNodes;
                l = e->child;
                r = e->parent;
            }
            right2left = this->mesh_vertices[l].p - this->mesh_vertices[r].p;
            left2right = this->mesh_vertices[r].p - this->mesh_vertices[l].p;
            auto iterator = vec->begin();
            while(iterator != vec->end()){
                node = *iterator;
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
                robustOrientation rightOri = Orient(this->mesh_vertices[right_child].p,
                                                    this->mesh_vertices[right_parent].p,
                                                    p, useRobustOrientatation);
                robustOrientation leftOri = Orient(this->mesh_vertices[left_child].p,
                                                   this->mesh_vertices[left_parent].p,
                                                   p, useRobustOrientatation);
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
                        //std::cout << "leving right" <<std::endl;
                        LineLineIntersectionNotCollinear(p, last_visible, last_point, this->evaluate_intersection(node.P), I);
                        out.push_back(I);
                    }
                    if(current_state == STATE::RIGHT and last_state == STATE::VISIBLE){
                        //std::cout << "leving to right " << int(iterator.base() - vec->begin().base()) << std::endl;

                        Point n = this->evaluate_intersection(last_node.P);
                        Point i;
                        LineLineIntersectionNotCollinear(p,
                                                         n,
                                                         this->mesh_vertices[e->parent].p,
                                                         this->mesh_vertices[e->child].p,
                                                         i);
                        Point vector = i - this->mesh_vertices[r].p;
                        if(abs(vector.x) > 1e-16){
                            currentLimitation = abs(vector.x / right2left.x);
                        }else if(abs(vector.y) > 1e-16){
                            currentLimitation = abs(vector.y / right2left.y);
                        }else{
                            currentLimitation = 0;
                        }
                        while(currentLimitation < iterator->root_R_order){
                            //std::cout << ":  " << iterator->OT1 << " | "<< iterator->OT2<< std::endl;
                            //std::cout << currentLimitation << " < " << iterator->TESTER << " | "<< e->INITED3<< std::endl;
                            //this->visualise_point(this->evaluate_intersection(iterator->P), 2, true);
                            iterator++;
                        }
                        iterator--;
                    }
                    if(current_state == STATE::LEFT){
                        //std::cout << "entering left" <<std::endl;
                        // entering left causes hanging intersection
                        segment_holder[0] = last_point;
                        segment_holder[1] = this->evaluate_intersection(node.P);
                        if(last_state == STATE::VISIBLE){
                            Point n = this->evaluate_intersection(last_node.P);
                            Point i;
                            LineLineIntersectionNotCollinear(p,
                                                             n,
                                                             this->mesh_vertices[e->parent].p,
                                                             this->mesh_vertices[e->child].p,
                                                             i);
                            Point vector = i - this->mesh_vertices[l].p;
                            if(abs(vector.x) > 1e-16){
                                currentLimitation = abs(vector.x / left2right.x);
                            }else if(abs(vector.y) > 1e-16){
                                currentLimitation = abs(vector.y / left2right.y);
                            }else{
                                currentLimitation = 0;
                            }

                            while(currentLimitation < iterator->root_L_order){
                                //std::cout << currentLimitation << " < " << iterator->TESTER << std::endl;
                                //this->visualise_point(this->evaluate_intersection(iterator->P), 2, true);
                                iterator++;
                            }
                            iterator--;
                        }
                    }
                    if(last_state == STATE::LEFT && current_state == STATE::VISIBLE){
                        //std::cout << "leving left" <<std::endl;
                        // leaving LEFT will trigger intersection with segment in memory
                        LineLineIntersectionNotCollinear(p, this->evaluate_intersection(node.P), segment_holder[0], segment_holder[1], I);
                        out.push_back(I);
                    }
                }

                if(current_state == STATE::VISIBLE){
                    //std::cout << "visible" <<std::endl;
                    last_visible = this->evaluate_intersection(node.P);
                    out.push_back(last_visible);
                }

                last_point = this->evaluate_intersection(iterator->P);
                last_node = *iterator;
                last_state = current_state;
                iterator++;
            }
        }
        return out;
    }

    std::vector<Point>
    Mesh::find_point_visibility_optim4(Point p){
        /**
         * @brief short desription.
         *
         * long description
         *
         * @param param1 param description.
         * @return return value description.
         */
        std::vector<Point> out;
        PointLocation pl = this->get_point_location(p);
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
        std::vector<OptimNode> *vec;
        std::vector<OptimNode> debugger;
        OptimNode node, last_node;

        int lastRight, lastLeft;
        int l,r;
        Point right2left;
        Point left2right;
        float currentLimitation;

        bool skip = false;
        for (Edge* e :edges) {
            if (pl.poly1 == e->rightPoly && e->leftOptNodes.size() > 0) {
                vec = &e->leftOptNodes;
                l = e->parent;
                r = e->child;
            }else if((pl.poly1 == e->leftPoly && e->rightOptNodes.size() > 0)) {
                vec = &e->rightOptNodes;
                l = e->child;
                r = e->parent;
            }
            right2left = this->mesh_vertices[l].p - this->mesh_vertices[r].p;
            left2right = this->mesh_vertices[r].p - this->mesh_vertices[l].p;
            auto iterator = vec->begin();
            while(iterator != vec->end()){

                node = *iterator;
                if(node.isAlwaysVisible) {
                    current_state = STATE::VISIBLE;
                }else {

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
                    robustOrientation rightOri = Orient(this->mesh_vertices[right_child].p,
                                                        this->mesh_vertices[right_parent].p,
                                                        p, useRobustOrientatation);
                    robustOrientation leftOri = Orient(this->mesh_vertices[left_child].p,
                                                       this->mesh_vertices[left_parent].p,
                                                       p, useRobustOrientatation);
                    if (rightOri != robustOrientation::kLeftTurn && leftOri != robustOrientation::kRightTurn) {
                        current_state = STATE::VISIBLE;
                    } else {
                        if (rightOri == robustOrientation::kLeftTurn) {
                            current_state = STATE::RIGHT;
                        } else {
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
                    if(current_state == STATE::RIGHT and last_state == STATE::VISIBLE){
                        Point n = this->evaluate_intersection(last_node.P);
                        Point i;
                        LineLineIntersectionNotCollinear(p,
                                                         n,
                                                         this->mesh_vertices[e->parent].p,
                                                         this->mesh_vertices[e->child].p,
                                                         i);
                        Point vector = i - this->mesh_vertices[r].p;
                        if(vector.x != 0){
                            currentLimitation = abs(vector.x / right2left.x);
                        }else if(vector.y != 0){
                            currentLimitation = abs(vector.y / right2left.y);
                        }else{
                            currentLimitation = 0;
                        }
                        while(currentLimitation < iterator->root_R_order){
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
                                                             this->mesh_vertices[e->parent].p,
                                                             this->mesh_vertices[e->child].p,
                                                             i);
                            Point vector = i - this->mesh_vertices[l].p;
                            if(vector.x != 0){
                                currentLimitation = abs(vector.x / left2right.x);
                            }else if(vector.y != 0){
                                currentLimitation = abs(vector.y / left2right.y);
                            }else{
                                currentLimitation = 0;
                            }
                            while(currentLimitation < iterator->root_L_order){
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
                last_point = this->evaluate_intersection(iterator->P);
                last_node = *iterator;
                last_state = current_state;
                iterator++;
            }
        }
        return out;
    }

    std::vector<Point>
    Mesh::find_point_visibility_TEA(Point p, double &T1, double &T2, double &T3) {
        /**
         * @brief Top-level function for finding point visibility using TEA.
         *
         * @param p 2D point - seeker/observer.
         * @param debug flag for possible debug mode.
         * @return List of points defining visibility polygon in CCW order.
         */
        //std::cout << "NEWPOINT" << std::endl;
        int num;
        std::vector<Point> result;
        visSize = 0;
        vis.clear();
        SearchPoint tmp;
        SearchNode* nodes = new edgevis::SearchNode[3];
        num = get_point_init_nodes(p, nodes, T1);
        if( num == -1){
            T2 = T1;
            T3 = T1;
            result.clear();
            return result;
        }
        visSize = vis.size();
        for(int i = 0; i < num; i++){
            vis.push_back(nodes[i].transitionR); // push first point
            expand_TEA(nodes[i], 0);
            vis.push_back(nodes[i].transitionL); // push last point
        }

        T2 = clock.TimeInSeconds();
        clock.Restart();

        b = this->evaluate_intersection(vis[0]); //not optimal 1st intersection is calculated twice
        for(auto n : vis){
            r = this->evaluate_intersection(n);
            if(b != r) result.push_back(r);
            b = r;
        }
        T3 = clock.TimeInSeconds();
        clock.Restart();
        vis.clear();
        free_points.clear();
        delete[] nodes;
        return result;
    }

    std::vector<Point>
    Mesh::find_point_visibility_PEA(Point p, double &T1, double &T2, double &T3) {
        /**
         * @brief Top-level function for finding point visibility using PEA.
         *
         * @param p 2D point - seeker/observer.
         * @param debug flag for possible debug mode.
         * @return List of points defining visibility polygon in CCW order.
         */
        int num;
        std::vector<Point> result;
        visSize = 0;
        vis.clear();
        SearchPoint tmp;
        SearchNode* nodes = new edgevis::SearchNode[this->max_poly_sides];
        num = get_point_init_nodes(p, nodes, T1);
        if( num == -1){
            T2 = T1;
            T3 = T1;
            result.clear();
            return result;
        }
        //std::cout << "s" <<std::endl;
        //this->reset_visu();
        //this->visualise_point(p, 0, true);
        //std::cout << p << std::endl;
        for(int i = 0; i < num; i++){
            //std::cout << i << std::endl;
            //if(i==2){
            //    this->reset_visu();
            //    this->visualise_point(p, 0, true);
            //    this->visualise_segment(this->evaluate_intersection(nodes[i].transitionR),
            //                            this->evaluate_intersection(nodes[i].transitionL), 0, 0.5);
            //    getchar();
            //}
            vis.push_back(nodes[i].transitionR); // push first point
            expand_PEA(nodes[i], 0);
            vis.push_back(nodes[i].transitionL); // push last point
        }
        //std::cout << "e" <<std::endl;
        T2 = clock.TimeInSeconds();
        clock.Restart();

        b = this->evaluate_intersection(vis[0]); //not optimal 1st intersection is calculated twice
        for(auto n : vis){
            r = this->evaluate_intersection(n);
            if(b != r) result.push_back(r);
            b = r;
        }
        T3 = clock.TimeInSeconds();
        clock.Restart();
        vis.clear();
        free_points.clear();
        delete[] nodes;
        return result;
    }

    void Mesh::expand_TEA(SearchNode &n, int level) {
        /**
         * @brief Recursive method for DFS-like TEA search algorithm.
         *
         * Given a single search node, we either return new points defining visibility polygon, or we
         * expand the visibility. Currently recursion stops only at obstacles. When expanding visibility
         * we look through the search node (from point over edge) for new search nodes which are then
         * processed in CCW order -> DFS (always take right most search node)
         *
         * @param n reference to current search node.
         * @param level counter for keeping track of recursion depth.
         * @return null - visibility is stored in class container.
         */
        if(n.nextPolygon == -1){
            vis.push_back(n.transitionR);
            vis.push_back(n.transitionL);
            return;
        }

        int num;
        if(level == TEAItems){
            TEAItems = TEAItems + 100;
            realloc_TEA_mem(TEAItems);
        }
        std::vector<SearchNode>& nodes = allocTEA[level].SearchNodes;
        num = this->expand_TEA_once(n, nodes);
        //if(num == 2) leveller = level;
        for(int i = 0; i < num; i++){
            expand_TEA(nodes[i], level+1);
        }
        return;
    }

    void Mesh::expand_PEA(SearchNode &n, int level) {
        /**
         * @brief Recursive method for DFS-like PEA search algorithm.
         *
         * Given a single search node, we either return new points defining visibility polygon, or we
         * expand the visibility. Currently recursion stops only at obstacles. When expanding visibility
         * we look through the search node (from point over edge) for new search nodes which are then
         * processed in CCW order -> DFS (always take right most search node)
         * Uses TEA expansion if expanding to triangle.
         *
         * @param n reference to current search node.
         * @param level counter for keeping track of recursion depth.
         * @return null - visibility is stored in class container.
         */
        if(n.nextPolygon == -1){
            vis.push_back(n.transitionR);
            vis.push_back(n.transitionL);
            return;
        }
        if(level == PEAItems){
            PEAItems = PEAItems + 100;
            //std::cout << "memTrigger PEA " << PEAItems << std::endl;
            realloc_PEA_mem(PEAItems);
        }

        int num = mesh_polygons[n.nextPolygon].vertices.size();
        //SearchNode* nodes = new edgevis::SearchNode[num - 1];
        std::vector<SearchNode>& nodes = allocPEA[level].SearchNodes;
        //std::cout << num << std::endl;
        if(num == 3){
            num = this->expand_TEA_once(n, nodes);
        }else{
            num = this->expand_PEA_once(n, nodes);
        }
        //for(int i = 0; i < num; i++){
        //    this->visualise_segment(this->evaluate_intersection(nodes[i].transitionR),
        //                            this->evaluate_intersection(nodes[i].transitionL), 0, 0.5);
        //    getchar();
        //}

        for(int i = 0; i < num; i++){
            expand_PEA(nodes[i], level+1);
        }
        //delete nodes;
        return;
    }

    int Mesh::expand_TEA_once(SearchNode &node, std::vector<SearchNode> &newNodes){
        init_temp_node(node);
        Point parent, right_child, left_child;
        int right_child_int, left_child_int;

        right_child_int = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        left_child_int = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        //std::cout << left_child_int << "("<< node.transitionL.isIntersection <<  ") | " << right_child_int << std::endl;
        right_child = this->mesh_vertices[right_child_int].p;
        left_child = this->mesh_vertices[left_child_int].p;
        parent = this->free_points[-node.rootR.p - 1];

        const Polygon &expander = this->mesh_polygons[node.nextPolygon];
        int S = expander.vertices.size();
        assert(S == 3);
        //int offset, num;
        //num = 0;
        //offset = normalise(expander, node.rightVertex, sortedV, sortedP);
        int a, b, c, A, B, C, num;
        num = 0;
        if (expander.vertices[0] == node.rightVertex)
        {
            // t1 = V[0], t2 = V[1], t3 = V[2]
            b = expander.vertices[0];
            c = expander.vertices[1];
            a = expander.vertices[2];

            B = expander.polygons[1];
            A = expander.polygons[0];
            C = expander.polygons[2];
        }
        else if (expander.vertices[0] == node.leftVertex)
        {
            b = expander.vertices[1];
            c = expander.vertices[2];
            a = expander.vertices[0];

            B = expander.polygons[2];
            A = expander.polygons[1];
            C = expander.polygons[0];
        }
        else
        {
            b = expander.vertices[2];
            c = expander.vertices[0];
            a = expander.vertices[1];

            B = expander.polygons[0];
            A = expander.polygons[2];
            C = expander.polygons[1];
        }

        /*      c
         *  C /   \  B
         *   /     \
         *  a---^---b
         *      A
         */
        SearchPoint rightIntersection, leftIntersection;

        robustOrientation rOri = Orient(parent, right_child, mesh_vertices[c].p, useRobustOrientatation);

        if(rOri == robustOrientation::kLeftTurn) {
            robustOrientation lOri = Orient(parent, left_child, mesh_vertices[c].p, useRobustOrientatation);
            if (lOri == robustOrientation::kRightTurn) {
                // In between
                //std::cout << " debug in"<<std::endl;
                if (node.transitionR.isIntersection) {
                    rightIntersection.isIntersection = true;
                    rightIntersection.i.a = node.rootR.p;
                    rightIntersection.i.b = right_child_int;
                    rightIntersection.i.c = b;
                    rightIntersection.i.d = c;
                } else {
                    rightIntersection.isIntersection = false;
                    rightIntersection.p = b;
                }
                if (node.transitionL.isIntersection) {
                    leftIntersection.isIntersection = true;
                    leftIntersection.i.a = node.rootR.p;
                    leftIntersection.i.b = left_child_int;
                    leftIntersection.i.c = c;
                    leftIntersection.i.d = a;
                } else {
                    leftIntersection.isIntersection = false;
                    leftIntersection.p = a;
                }
                temp.transitionR = rightIntersection;
                temp.transitionL.isIntersection = false;
                temp.transitionL.p = c;
                temp.nextPolygon = B;
                temp.leftVertex = c;
                temp.rightVertex = b;
                newNodes[num++] = temp;

                temp.transitionL = leftIntersection;
                temp.transitionR.isIntersection = false;
                temp.transitionR.p = c;
                temp.nextPolygon = C;
                temp.leftVertex = a;
                temp.rightVertex = c;
                newNodes[num++] = temp;
            } else {
                // Left
                //std::cout << " debug left"<<std::endl;
                if (node.transitionR.isIntersection) {
                    rightIntersection.isIntersection = true;
                    rightIntersection.i.a = node.rootR.p;
                    rightIntersection.i.b = right_child_int;
                    rightIntersection.i.c = b;
                    rightIntersection.i.d = c;
                } else {
                    rightIntersection.isIntersection = false;
                    rightIntersection.p = b;
                }
                if (lOri == robustOrientation::kCollinear) {
                    leftIntersection.isIntersection = false;
                    leftIntersection.p = c;
                } else {
                    leftIntersection.isIntersection = true;
                    leftIntersection.i.a = node.rootR.p;
                    leftIntersection.i.b = left_child_int;
                    leftIntersection.i.c = b;
                    leftIntersection.i.d = c;
                }
                temp.transitionR = rightIntersection;
                temp.transitionL = leftIntersection;
                temp.nextPolygon = B;
                temp.leftVertex = c;
                temp.rightVertex = b;
                newNodes[num++] = temp;
            }
        } else {
            // Right
            //std::cout << " debug right"<<std::endl;
            if (rOri == robustOrientation::kCollinear) {
                rightIntersection.isIntersection = false;
                rightIntersection.p = c;
            } else {
                rightIntersection.isIntersection = true;
                rightIntersection.i.a = node.rootR.p;
                rightIntersection.i.b = right_child_int;
                rightIntersection.i.c = c;
                rightIntersection.i.d = a;
                //std::cout << "r1" << std::endl;
            }
            if (node.transitionL.isIntersection) {
                leftIntersection.isIntersection = true;
                leftIntersection.i.a = node.rootR.p;
                leftIntersection.i.b = left_child_int;
                leftIntersection.i.c = c;
                leftIntersection.i.d = a;
                //std::cout << "r3" <<std::endl;
            } else {
                leftIntersection.isIntersection = false;
                leftIntersection.p = a;
                //std::cout << "r2" << std::endl;
            }

            temp.transitionR = rightIntersection;
            temp.transitionL = leftIntersection;
            temp.nextPolygon = C;
            temp.leftVertex = a;
            temp.rightVertex = c;
            newNodes[num++] = temp;
        }

        return num;
    }

    int Mesh::expand_PEA_once(SearchNode& node, std::vector<SearchNode> &newNodes){
        // Temporary searchnode object for creating new nodes
        init_temp_node(node);
        int right_child, left_child;
        Point parent;

        right_child = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        left_child = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        parent = this->free_points[-node.rootR.p - 1];

        const Polygon &expander = this->mesh_polygons[node.nextPolygon];
        const std::vector<int> &V = expander.vertices;
        const std::vector<int> &P = expander.polygons;
        int S = expander.vertices.size();

        int offset;
        offset = normalise(expander, node.rightVertex);
        int i;
        uint8_t rCheck = 0;
        uint8_t lCheck = 0;
        // last visible point when going ccw -> left point or cw -> right point
        int right_visible, left_visible, visible;
        bool right_collinear, left_collinear;

        find_visible_binary_tree(node, offset, &right_visible, &left_visible, &right_collinear, &left_collinear);
        //std::cout << right_visible << " | " << left_visible << std::endl;

        SearchPoint right_intersection, left_intersection;
        int count = 0;
        if(right_visible == offset){
            right_intersection.p = V[right_visible];
            right_intersection.isIntersection = false;
        }else if (right_collinear) {
            right_intersection.p = V[right_visible];
            right_intersection.isIntersection = false;
        } else {
            right_intersection.isIntersection = true;
            right_intersection.i.a = node.rootR.p;
            right_intersection.i.b = right_child;
            right_intersection.i.c = V[right_visible];
            right_intersection.i.d = V[right_visible > 0 ? right_visible-1 : S-1];
        }

        if(left_visible == (offset+S-1) % S){
            left_intersection.p = V[left_visible];
            left_intersection.isIntersection = false;
        }else if (left_collinear) {
            left_intersection.p = V[left_visible];
            left_intersection.isIntersection = false;
        } else {
            left_intersection.isIntersection = true;
            left_intersection.i.a = node.rootR.p;
            left_intersection.i.b = left_child;
            left_intersection.i.c = V[left_visible];
            left_intersection.i.d = V[(left_visible+1)% S];
        }


        i = right_visible;
        int newI;
        if(right_intersection.isIntersection) {
            i = i == 0 ? S - 1 : i - 1;
            temp.transitionR = right_intersection;
        }else{
            temp.transitionR = right_intersection;
        }

        while(i != left_visible){
            newI = (i + 1) % S;
            temp.transitionL.p = V[newI];
            temp.transitionL.isIntersection = false;
            temp.nextPolygon = P[newI];
            temp.leftVertex = V[newI];
            temp.rightVertex = V[i];
            newNodes[count++] = temp;
            i = newI;
            temp.transitionR.p = V[i];
            temp.transitionR.isIntersection = false;
        }
        if(left_intersection.isIntersection) {
            newI = (i + 1) % S;
            temp.transitionL = left_intersection;
            temp.nextPolygon = P[newI];
            temp.leftVertex = V[newI];
            temp.rightVertex = V[i];
            newNodes[count++] = temp;
        }
        return count;
    }

    void
    Mesh::realloc_TEA_mem(int items){
        int currentSize = allocTEA.capacity();
        allocTEA.resize(items);
        for(int i = currentSize; i<items; i++){
            allocTEA[i].SearchNodes.resize(2);
        }
        return;
    }

    void
    Mesh::realloc_PEA_mem(int items){
        int currentSize = allocPEA.capacity();
        allocPEA.resize(items);
        for(int i = currentSize; i<items; i++){
            allocPEA[i].SearchNodes.resize(max_poly_sides);
        }
        return;
    }
}