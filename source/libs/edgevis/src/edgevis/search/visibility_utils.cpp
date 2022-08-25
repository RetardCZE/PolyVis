#include "edgevis/search/visibility_utils.h"
#include <vector>

namespace edgevis
{
    int
    normalise(const Polygon& P, int transition_R, std::vector<int>* sorted_vertices, std::vector<int>* sorted_polygons){
        sorted_vertices->clear();
        sorted_polygons->clear();
        int S = P.vertices.size();
        int offset = 0;
        int current = P.vertices[offset];
        while(current != transition_R){
            current = P.vertices[++offset];
        }

        for(int i = 0; i < S; i++){
            sorted_vertices->push_back(P.vertices[(i+offset) % S]);
            sorted_polygons->push_back(P.polygons[(i+offset) % S]);
        }

        return offset;
    }

    bool
    is_on_segment(Point A, Point B, Point C){
        Point V = B-A;
        Point T = C-A;
        if(C == A || C == B){
            return true;
        }
        double dx, dy;
        dx = T.x / V.x;
        dy = T.y / V.y;
        if(V.x == 0)
            dx = dy;
        if(V.y == 0)
            dy = dx;
        if (EPSILON > std::abs(dy - dx) && dx <= 1+ EPSILON && dx >= 0 - EPSILON && dy <= 1 + EPSILON && dy >= 0 - EPSILON)
            return true;
        return false;
    }

/*
    void
    recompute_end_roots(SearchNode &node, OptimNode &o) {
        SearchNode* parent;
        Point pivot_L, pivot_R;
        Point parent_intersection, child_intersection;
        parent = node.predecessor;
        if(parent) {
            while (parent) {
                parent_intersection = line_intersect(o.P,
                                                     parent->transitionR,
                                                     o.root_L,
                                                     o.root_R);


                child_intersection = line_intersect(o.P,
                                                    parent->transitionL,
                                                    o.root_R,
                                                    o.root_L);

                if (is_on_segment(o.root_R, o.root_L, parent_intersection) && node.rootL != node.transitionL) {
                    parent_intersection == parent_intersection ? o.root_R = parent_intersection : o.root_R;
                    o.pivot_R = parent->transitionR;
                }

                if (is_on_segment(o.root_R, o.root_L, child_intersection) && node.rootR != node.transitionR) {
                    child_intersection == child_intersection ? o.root_L = child_intersection : o.root_L;
                    o.pivot_L = parent->transitionL;
                }

                parent = parent->predecessor;
            }
        }else{
            o.pivot_R = node.rootR;
            o.pivot_L = node.rootL;
            o.root_R = node.rootR;
            o.root_L = node.rootL;
        }

    }
*/
    SearchNode
    init_temp_node(SearchNode& node){
        SearchNode temp;
        temp.comingFrom = node.nextPolygon;
        temp.rootR = node.rootR;
        temp.rootL = node.rootL;
        temp.predecessor = &node;
        return temp;
    }

}