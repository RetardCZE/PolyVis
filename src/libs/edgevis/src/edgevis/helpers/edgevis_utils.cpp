#include "edgevis/helpers/edgevis_utils.h"
#include <vector>

namespace edgevis
{
    int
    normalise(const Polygon& P, int transition_R, int *sorted_vertices, int *sorted_polygons){
        int S = P.vertices.size();
        int offset = 0;
        int current = P.vertices[offset];
        while(current != transition_R){
            current = P.vertices[++offset];
        }

        int j = 0;
        for(int i = 0; i < S; i++){
            sorted_vertices[j] = (P.vertices[(i+offset) % S]);
            sorted_polygons[j++] = (P.polygons[(i+offset) % S]);
        }
        return offset;
    }



    int
    Mesh::find_visible(SearchNode &node, int *right_visible, int *left_visible, int vCount) {
        /*
         * For both PEA and PEA-E - common linear search -> optimize to binary tree
         */
        const int S = vCount;
        int i;
        int right_child, left_child;
        Point right_parent, left_parent;
        right_child = node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p;
        left_child = node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p;
        if (node.rootR.p < 0) {
            right_parent = this->free_points[-node.rootL.p - 1];
        } else {
            right_parent = this->mesh_vertices[node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p].p;
        }
        if (node.rootR.p < 0) {
            left_parent = this->free_points[-node.rootR.p - 1];
        } else {
            left_parent = this->mesh_vertices[node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p].p;
        }

        *right_visible = S-1;
        *left_visible = 0;
        for(i = 0; i<S; i++){
            if (Orient(right_parent,
                       this->mesh_vertices[right_child].p,
                       this->mesh_vertices[sortedV[S - 1 - i]].p, useRobustOrientatation) == robustOrientation::kRightTurn){
                if( i == 0) {
                    std::cerr << "WARNING POLYGON NOT VISIBLE - RIGHT" << std::endl;
                }
                break;
            }
            *right_visible = S-1-i;
        }
        for(i = 0; i<S; i++){
            if (Orient(left_parent,
                       this->mesh_vertices[left_child].p,
                       this->mesh_vertices[sortedV[i]].p, useRobustOrientatation) == robustOrientation::kLeftTurn){
                if( i == 0) {
                    std::cerr << "WARNING POLYGON NOT VISIBLE - LEFT" << std::endl;
                }
                break;
            }
            *left_visible = i;
        }
        return *left_visible - *right_visible;
    }

    SearchNode
    init_temp_node(SearchNode& node){
        SearchNode temp;
        temp.comingFrom = node.nextPolygon;
        temp.rootR = node.rootR;
        temp.rootL = node.rootL;
        temp.rightRootVertex = node.rightRootVertex;
        temp.leftRootVertex = node.leftRootVertex;
        temp.predecessor = &node;
        return temp;
    }

}