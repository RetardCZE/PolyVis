#include "edgevis/structs/mesh.h"

namespace edgevis{
    void
    Mesh::expand(SearchNode &node, std::vector<SearchNode> &visibility, bool side, int level, bool draw) {
        if(node.nextPolygon == -1){
            visibility.push_back(node);
            return;
        }
        int num;

        SearchNode* nodes = new edgevis::SearchNode[this->max_poly_sides + 2];
        deleteQueue.push_back(nodes);
        num = this->expand_forward(node, nodes);

        for(int i = 0; i < num; i++){
            this->back_propagation(nodes[i]);
            expand(nodes[i], visibility, side, level + 1, draw);
        }
        return;
    }

    std::vector<SearchNode>
    Mesh::find_edge_visibility(int edge, bool side, bool draw) {
        current_edge = this->mesh_edges[edge];
        int num;
        std::vector<SearchNode> vis;

        SearchPoint tmp;
        SearchNode* nodes = new edgevis::SearchNode[this->max_poly_sides + 2];
        deleteQueue.push_back(nodes);
        num = this->get_edge_init_nodes(this->mesh_edges[edge], side, nodes);
        for(int i = 0; i < num; i++){
            expand(nodes[i], vis, side, 0, draw);
        }
        return vis;
    }

    int
    Mesh::expand_forward(SearchNode &node, SearchNode *newNodes) {
        // Temporary searchnode object for creating new nodes

        init_temp_node(node);
        // right visibility border

        int right_parent, right_child, left_parent, left_child;
        right_child = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        right_parent = node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p;
        // left visibility border
        left_child = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        left_parent = node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p;

        const Polygon &expander = this->mesh_polygons[node.nextPolygon];
        int S = expander.vertices.size();

        int offset;
        offset = normalise(expander, node.rightVertex, sortedV, sortedP);

        int i;
        uint8_t rCheck = 0;
        uint8_t lCheck = 0;
        // last visible point when going ccw -> left point or cw -> right point
        int right_visible, left_visible;
        int visible = find_visible(node, &right_visible, &left_visible, S);
        // std::cout << left_visible << " | " << right_visible << std::endl;
        SearchPoint right_intersection, left_intersection;
        int count = 0;

        if (Orient(this->mesh_vertices[right_parent].p,
                   this->mesh_vertices[right_child].p,
                   this->mesh_vertices[sortedV[right_visible]].p, useRobustOrientatation)
            ==
            robustOrientation::kCollinear) {
            right_intersection.p = sortedV[right_visible];
            right_intersection.isIntersection = false;
        } else {
            right_intersection.isIntersection = true;
            right_intersection.i.a = right_parent;
            right_intersection.i.b = right_child;
            right_intersection.i.c = sortedV[right_visible];
            right_intersection.i.d = sortedV[right_visible-1];
        }

        if (Orient(this->mesh_vertices[left_parent].p,
                   this->mesh_vertices[left_child].p,
                   this->mesh_vertices[sortedV[left_visible]].p, useRobustOrientatation)
                                ==
            robustOrientation::kCollinear) {
            left_intersection.isIntersection = false;
        } else {
            left_intersection.isIntersection = true;
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

    void
    Mesh::back_propagation(edgevis::SearchNode &node) {
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
                    robustOrientation ori = Orient(this->mesh_vertices[right_parent].p,
                                                   this->mesh_vertices[root_dir].p,
                                                   this->mesh_vertices[right_child].p, useRobustOrientatation);
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
                    robustOrientation ori = Orient(this->mesh_vertices[left_parent].p,
                                                   this->mesh_vertices[root_dir].p,
                                                   this->mesh_vertices[left_child].p, useRobustOrientatation);
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
    Mesh::get_edge_init_nodes(Edge edge, bool side, edgevis::SearchNode *initNodes) {
        // side - true = transitionR, false = transitionL
        Polygon expander;
        SearchNode temp;
        temp.predecessor = NULL;
        temp.rootL.isIntersection = false;
        temp.rootR.isIntersection = false;
        temp.transitionR.isIntersection = false;
        temp.transitionL.isIntersection = false;
        int count = 0;
        int S;
        if(side) {
            if(edge.rightPoly == -1) return 0;
            expander = this->mesh_polygons[edge.rightPoly];
            S = expander.vertices.size();
            normalise(expander, edge.parent, sortedV, sortedP);
            temp.rootR.p = edge.parent;
            temp.rootL.p = edge.child;
            temp.rightRootVertex = edge.parent;
            temp.leftRootVertex = edge.child;
            temp.comingFrom = edge.rightPoly;
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = this->mesh_polygons[edge.leftPoly];
            S = expander.vertices.size();
            normalise(expander, edge.child, sortedV, sortedP);
            temp.rootR.p = edge.child;
            temp.rootL.p = edge.parent;
            temp.rightRootVertex = edge.child;
            temp.leftRootVertex = edge.parent;
            temp.comingFrom = edge.leftPoly;
        }

        for(int i = 0; i < S-1; i++){
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
    Mesh::find_arbitrary_edge_visibility(int edgeParent, int edgeChild, std::vector<SearchNode> &leftVis,
                                                   std::vector<SearchNode> &rightVis) {
        Edge e;
        e.parent = edgeParent;
        e.child = edgeChild;
        e.leftPoly = e.rightPoly = NULL;
        int numLeft;
        int numRight;

        SearchNode* leftNodes = new edgevis::SearchNode[this->max_poly_sides + 2];
        SearchNode* rightNodes = new edgevis::SearchNode[this->max_poly_sides + 2];
        this->get_arbitrary_edge_init_nodes(e, numRight, numLeft, leftNodes, rightNodes);

        std::vector<Point> lPoints;
        if(numLeft){
            for(int i = 0; i < numLeft; i++){
                expand(leftNodes[i], leftVis, false, 0, false); // here side should be deprecated
            }
        }
        if(numRight){
            for(int i = 0; i < numRight; i++){
                expand(rightNodes[i], rightVis, true, 0, false); // here side should be deprecated
            }
        }
        return;
    }


    void
    Mesh::get_arbitrary_edge_init_nodes(Edge edge, int &rightCount, int &leftCount,
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

        for(auto p :  this->mesh_vertices[edge.parent].polygons) {
            polygons.push_back(p);
        }

        while(polygons.size()){
            int p = polygons.front();
            used.push_back(p);
            polygons.erase(polygons.begin());
            if(p == -1) continue;

            show = false;
            for(auto e : this->mesh_polygons[p].edges){
                if(this->mesh_edges[e] == edge) continue; // non oriented equality
                Point point;
                uint8_t check = SegmentSegmentIntersectionGeneral(this->mesh_vertices[edge.parent].p,
                                                                  this->mesh_vertices[edge.child].p,
                                                                  this->mesh_vertices[this->mesh_edges[e].parent].p,
                                                                  this->mesh_vertices[this->mesh_edges[e].child].p,
                                                                  point, useRobustOrientatation);
                if(check==0){
                    show = true;
                    is_used = false;
                    if(this->mesh_edges[e].leftPoly == p){
                        new_p = this->mesh_edges[e].rightPoly;
                    }else{
                        new_p = this->mesh_edges[e].leftPoly;
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

                }else if(check == 2 && point != this->mesh_vertices[edge.parent].p){
                    show = true;
                    if(point != this->mesh_vertices[edge.child].p){
                        for(auto new_p : this->mesh_vertices[e].polygons){
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
        int current_point = this->mesh_edges[current_edge].child;
        std::vector<int> orderedVerts, orderedLeftPoly, orderedRightPoly;
        edges.erase(edges.begin());
        orderedVerts.push_back(current_point);
        orderedLeftPoly.push_back(this->mesh_edges[current_edge].leftPoly);
        orderedRightPoly.push_back(this->mesh_edges[current_edge].rightPoly);
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
                if(current_point == this->mesh_edges[edges[i]].parent){
                    current_edge = edges[i];
                    current_point = this->mesh_edges[current_edge].child;
                    orderedVerts.push_back(current_point);
                    edges.erase(edges.begin()+i);
                    orderedLeftPoly.push_back(this->mesh_edges[current_edge].leftPoly);
                    orderedRightPoly.push_back(this->mesh_edges[current_edge].rightPoly);
                    disconnected = false;
                    break;
                }else if(current_point == this->mesh_edges[edges[i]].child ) {
                    current_edge = edges[i];
                    current_point = this->mesh_edges[current_edge].parent;
                    orderedVerts.push_back(current_point);
                    edges.erase(edges.begin() + i);
                    orderedLeftPoly.push_back(this->mesh_edges[current_edge].rightPoly);
                    orderedRightPoly.push_back(this->mesh_edges[current_edge].leftPoly);
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
            winding = Orient(this->mesh_vertices[orderedVerts[beforeP]].p,
                             this->mesh_vertices[orderedVerts[p]].p,
                             this->mesh_vertices[orderedVerts[c]].p, useRobustOrientatation);
            if(beforeP == c){
                break;
            }
        }
        if(winding == robustOrientation::kCollinear){
            beforeP = beforeP > 0 ? beforeP - 1 : orderedVerts.size()-1;
            winding = Orient(this->mesh_vertices[orderedVerts[beforeP]].p,
                             this->mesh_vertices[orderedVerts[c]].p,
                             this->mesh_vertices[orderedVerts[p]].p, useRobustOrientatation);
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
            tmp.transitionL.p = orderedVerts[(i + 1) % S];
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
                    ori = Orient(this->mesh_vertices[tmp.transitionR.p].p,
                                 this->mesh_vertices[tmp.rootL.p].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
                    if(ori==robustOrientation::kLeftTurn){
                        // root is limited
                        tmp.rootL.isIntersection = true;
                        tmp.rootL.i.b = tmp.transitionR.p;
                        tmp.rootL.i.a = orderedVerts[j];
                        tmp.rootL.i.c = tmp.leftRootVertex;
                        tmp.rootL.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(this->mesh_vertices[tmp.rootL.i.b].p,
                                 this->mesh_vertices[tmp.rootL.i.a].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
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
                    ori = Orient(this->mesh_vertices[tmp.transitionL.p].p,
                                 this->mesh_vertices[tmp.rootR.p].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
                    if(ori==robustOrientation::kRightTurn){
                        // root is limited
                        tmp.rootR.isIntersection = true;
                        tmp.rootR.i.b = tmp.transitionL.p;
                        tmp.rootR.i.a = orderedVerts[j];
                        tmp.rootR.i.c = tmp.leftRootVertex;
                        tmp.rootR.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(this->mesh_vertices[tmp.rootR.i.b].p,
                                 this->mesh_vertices[tmp.rootR.i.a].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
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
            tmp.transitionL.p = orderedVerts[(i + 1) % S];
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
                    ori = Orient(this->mesh_vertices[tmp.transitionR.p].p,
                                 this->mesh_vertices[tmp.rootL.p].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
                    if(ori==robustOrientation::kLeftTurn){
                        // root is limited
                        tmp.rootL.isIntersection = true;
                        tmp.rootL.i.b = tmp.transitionR.p;
                        tmp.rootL.i.a = orderedVerts[j];
                        tmp.rootL.i.c = tmp.leftRootVertex;
                        tmp.rootL.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(this->mesh_vertices[tmp.rootL.i.b].p,
                                 this->mesh_vertices[tmp.rootL.i.a].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
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
                    ori = Orient(this->mesh_vertices[tmp.transitionL.p].p,
                                 this->mesh_vertices[tmp.rootR.p].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
                    if(ori==robustOrientation::kRightTurn){
                        // root is limited
                        tmp.rootR.isIntersection = true;
                        tmp.rootR.i.b = tmp.transitionL.p;
                        tmp.rootR.i.a = orderedVerts[j];
                        tmp.rootR.i.c = tmp.leftRootVertex;
                        tmp.rootR.i.d = tmp.rightRootVertex;
                    }
                }else{
                    ori = Orient(this->mesh_vertices[tmp.rootR.i.b].p,
                                 this->mesh_vertices[tmp.rootR.i.a].p,
                                 this->mesh_vertices[orderedVerts[j]].p, useRobustOrientatation);
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

}