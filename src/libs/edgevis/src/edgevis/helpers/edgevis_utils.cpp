#include "edgevis/helpers/edgevis_utils.h"
#include <vector>

namespace edgevis
{
    int
    normalise(const Polygon &P, int transition_R) {
        int offset = 0;
        int current = P.vertices[offset];
        while(current != transition_R){
            current = P.vertices[++offset];
        }
        return offset;
        //int j = 0;
        //for(int i = 0; i < S; i++){
        //    sorted_vertices[j] = (P.vertices[(i+offset) % S]);
        //    sorted_polygons[j++] = (P.polygons[(i+offset) % S]);
        //}
        //return offset;
    }

    int
    Mesh::find_visible_binary_tree(SearchNode &node, const int offset, int *right_visible, int *left_visible,
                                   bool *right_collinear, bool *left_collinear) {
        const Polygon &expander = this->mesh_polygons[node.nextPolygon];
        const std::vector<int> &V = expander.vertices;
        const int S = expander.vertices.size();

        Point right_child, left_child;
        Point right_parent, left_parent;
        right_child = this->mesh_vertices[node.transitionR.isIntersection ? node.transitionR.i.b : node.transitionR.p].p;
        left_child = this->mesh_vertices[node.transitionL.isIntersection ? node.transitionL.i.b : node.transitionL.p].p;

        if (node.rootR.p < 0) {
            right_parent = this->free_points[-node.rootL.p - 1];
            left_parent = this->free_points[-node.rootR.p - 1];
        } else {
            right_parent = this->mesh_vertices[node.rootL.isIntersection ? node.rootL.i.a : node.rootL.p].p;
            left_parent = this->mesh_vertices[node.rootR.isIntersection ? node.rootR.i.a : node.rootR.p].p;
        }

        int r,m,l,best;
        r = 0;
        l = S - 1;
        best = l;
        robustOrientation Ori;
        *right_collinear = false;
        while(r <= l){
            if (l == 1 && r == 0) {
                m = 1;  // Set m to 1 for this specific case
            } else {
                m = r + (l - r) / 2;
            }
            //std::cout << l << " | " << m << " | " << r << std::endl;
            Ori = Orient(right_parent,
                         right_child,
                         this->mesh_vertices[V[(m+offset) % S]].p, useRobustOrientatation);
            if(Ori == robustOrientation::kCollinear){
                best = m;
                *right_collinear = true;
                break;
            }else if(Ori == robustOrientation::kRightTurn){
                r = m + 1;
            }else{
                l = m - 1;
                best = m;
            }
        }
        *right_visible = (best+offset) % S;

        r = 0;
        l = S - 1;
        best = r;
        *left_collinear = false;
        while( r <= l){
            if (l == S - 1 && r == S - 2) {
                m = S - 2;  // Set m to 1 for this specific case
            } else {
                m = r + (l - r) / 2;
            }
            Ori = Orient(left_parent,
                         left_child,
                         this->mesh_vertices[V[(m+offset) % S]].p, useRobustOrientatation);
            if(Ori == robustOrientation::kCollinear){
                best = m;
                *left_collinear = true;
                break;
            }else if(Ori == robustOrientation::kRightTurn){
                r = m+1;
                best = m;
            }else{
                l = m-1;
            }
        }
        *left_visible = (best+offset) % S;

        return *left_visible - *right_visible;
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

    void
    Mesh::init_temp_node(SearchNode &node){
        temp.comingFrom = node.nextPolygon;
        temp.rootR = node.rootR;
        temp.rootL = node.rootL;
        temp.rightRootVertex = node.rightRootVertex;
        temp.leftRootVertex = node.leftRootVertex;
        temp.predecessor = &node;
    }

}