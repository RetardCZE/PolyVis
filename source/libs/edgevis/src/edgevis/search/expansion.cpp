#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/structs/edge.h"
#include "edgevis/helpers/geometry.h"
#include <vector>

namespace edgevis
{
    bool
    is_observable(Point left_parent, Point left_child, Point right_parent, Point right_child, Point p){
        Orientation L = get_orientation(left_parent, left_child, p);
        Orientation R = get_orientation(right_parent, right_child, p);
        bool a = (L == Orientation::CW || L == Orientation::COLLINEAR);
        bool b = (R == Orientation::CCW || R == Orientation::COLLINEAR);
        return a && b;
    }

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

    int
    get_edge_init_nodes(Edge edge, bool side, const Mesh& mesh, SearchNode* initNodes){
        // side - true = child_R, false = child_L
        Polygon expander;
        SearchNode temp;
        temp.predecessor = NULL;
        int count = 0;
        std::vector<int> sortedV, sortedP;

        if(side) {
            if(edge.rightPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.rightPoly];
            normalise(expander, edge.parent, &sortedV, &sortedP);
            temp.root_R = mesh.mesh_vertices[edge.parent].p;
            temp.root_L = mesh.mesh_vertices[edge.child].p;
            temp.coming_from = edge.rightPoly;
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.leftPoly];
            normalise(expander, edge.child, &sortedV, &sortedP);
            temp.root_R = mesh.mesh_vertices[edge.parent].p;
            temp.root_L = mesh.mesh_vertices[edge.child].p;
            temp.coming_from = edge.leftPoly;
        }

        for(int i = 0; i < sortedV.size()-1; i++){
            temp.child_R = mesh.mesh_vertices[sortedV[i]].p;
            temp.child_L = mesh.mesh_vertices[sortedV[i+1]].p;
            temp.right_vertex = sortedV[i];
            temp.left_vertex = sortedV[i+1];
            temp.next_polygon = sortedP[i+1];
            initNodes[count++] = temp;
        }

        return count;
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

    void
    recompute_roots(SearchNode &node){
        Point parent_intersection, child_intersection;
        SearchNode* parent;
        parent = node.predecessor;
        while(parent){
            parent_intersection = line_intersect(node.child_L,
                                                 parent->child_R,
                                                 node.root_L,
                                                 node.root_R);


            child_intersection = line_intersect(node.child_R,
                                                parent->child_L,
                                                node.root_R,
                                                node.root_L);

            if(is_on_segment(node.root_R, node.root_L, parent_intersection) && node.root_L != node.child_L){
                parent_intersection == parent_intersection ? node.root_R = parent_intersection : node.root_R;
            }

            if(is_on_segment(node.root_R, node.root_L, child_intersection) && node.root_R != node.child_R){
                child_intersection == child_intersection ? node.root_L = child_intersection : node.root_L;
            }

            parent = parent->predecessor;
        }
    }

    void
    recompute_end_roots(SearchNode &node, OptimNode &o) {
        SearchNode* parent;
        Point pivot_L, pivot_R;
        Point parent_intersection, child_intersection;
        parent = node.predecessor;
        if(parent) {
            while (parent) {
                parent_intersection = line_intersect(o.P,
                                                     parent->child_R,
                                                     o.root_L,
                                                     o.root_R);


                child_intersection = line_intersect(o.P,
                                                    parent->child_L,
                                                    o.root_R,
                                                    o.root_L);

                if (is_on_segment(o.root_R, o.root_L, parent_intersection) && node.root_L != node.child_L) {
                    parent_intersection == parent_intersection ? o.root_R = parent_intersection : o.root_R;
                    o.pivot_R = parent->child_R;
                }

                if (is_on_segment(o.root_R, o.root_L, child_intersection) && node.root_R != node.child_R) {
                    child_intersection == child_intersection ? o.root_L = child_intersection : o.root_L;
                    o.pivot_L = parent->child_L;
                }

                parent = parent->predecessor;
            }
        }else{
            o.pivot_R = node.root_R;
            o.pivot_L = node.root_L;
            o.root_R = node.root_R;
            o.root_L = node.root_L;
        }

    }

    SearchNode
    init_temp_node(SearchNode& node){
        SearchNode temp;
        temp.coming_from = node.next_polygon;
        temp.root_R = node.root_R;
        temp.root_L = node.root_L;
        temp.predecessor = &node;
        return temp;
    }

    int
    find_visible(const Mesh& mesh, SearchNode& node, std::vector<int>& sorted_vertices, int* right_visible, int* left_visible){
        const int S = sorted_vertices.size();
        int i;
        // right line
        Point right_child = node.child_R;
        Point right_parent = node.root_L;
        // left line
        Point left_child = node.child_L;
        Point left_parent = node.root_R;
        *right_visible = S-1;
        *left_visible = 0;
        for(i = 0; i<S; i++){
            if (!is_observable(right_child, left_child, right_parent, right_child, mesh.mesh_vertices[sorted_vertices[S-1-i]].p)){
                break;
            }
            *right_visible = S-1-i;
        }
        for(i = 0; i<S; i++){
            if (!is_observable(left_parent, left_child, left_child, right_child, mesh.mesh_vertices[sorted_vertices[i]].p)){
                break;
            }
            *left_visible = i;
        }
        return *left_visible - *right_visible;
    }

    int
    expand_searchnode(SearchNode& node, const Mesh& mesh, SearchNode* newNodes) {
        // Temporary searchnode object for creating new nodes
        SearchNode temp = init_temp_node(node);

        // right line
        Point right_child = node.child_R;
        Point right_parent = node.root_L;

        // left line
        Point left_child = node.child_L;
        Point left_parent = node.root_R;

        const Polygon &expander = mesh.mesh_polygons[node.next_polygon];
        std::vector<int> sortedV, sortedP;

        int offset;
        offset = normalise(expander, node.right_vertex, &sortedV, &sortedP);
        const int S = sortedV.size();

        int i;
        int right_visible, left_visible;
        int visible = find_visible(mesh, node, sortedV, &right_visible, &left_visible);

        Point right_intersection, left_intersection;
        int count = 0;

        if (get_orientation(right_child, right_parent, mesh.mesh_vertices[sortedV[right_visible]].p) ==
            Orientation::COLLINEAR ||
            right_visible == 0) {
            right_intersection = mesh.mesh_vertices[sortedV[right_visible]].p;
        } else {
            right_intersection = line_intersect(mesh.mesh_vertices[sortedV[right_visible - 1]].p,
                                                mesh.mesh_vertices[sortedV[right_visible]].p,
                                                right_parent, right_child);
        }

        if (get_orientation(left_parent, left_child, mesh.mesh_vertices[sortedV[left_visible]].p) ==
            Orientation::COLLINEAR ||
            left_visible == S - 1) {
            left_intersection = mesh.mesh_vertices[sortedV[left_visible]].p;
        } else {
            left_intersection = line_intersect(mesh.mesh_vertices[sortedV[left_visible]].p,
                                               mesh.mesh_vertices[sortedV[left_visible + 1]].p,
                                               left_parent, left_child);
        }
        if (!(left_intersection == left_intersection && right_intersection == right_intersection)) {
            /*
            std::cout << left_intersection << " | " << right_intersection << std::endl;
             */
            Point a = mesh.mesh_vertices[sortedV[left_visible]].p;
            Point b = mesh.mesh_vertices[sortedV[left_visible + 1]].p;
            Point c = left_parent;
            Point d = left_child;
            const Point ab = b - a;
            /*
            std::cout << a << " | "<< ab << " | "  <<(c - a) << " | "  <<(d - a)<< " | " << (d - c) <<std::endl;

            std::cout << ( a + ab * ((c - a) * (d - a))) << " | " << (ab * (d - c)) << std::endl;
             */
            left_intersection = ( a + ab * ((c - a) * (d - a)));
        }
        i = right_visible;
        if(right_intersection != mesh.mesh_vertices[sortedV[right_visible]].p) {
            i--;
            temp.child_R = right_intersection;
        }else{
            temp.child_R = mesh.mesh_vertices[sortedV[i]].p;
        }
        while(i < left_visible){
            temp.child_L = mesh.mesh_vertices[sortedV[i + 1]].p;
            temp.next_polygon = sortedP[i + 1];
            temp.left_vertex = sortedV[i+1];
            temp.right_vertex = sortedV[i];
            newNodes[count++] = temp;
            i++;
            temp.child_R = mesh.mesh_vertices[sortedV[i]].p;
        }

        if(left_intersection != mesh.mesh_vertices[sortedV[left_visible]].p) {
            temp.child_L = left_intersection;
            temp.next_polygon = sortedP[i + 1];
            temp.left_vertex = sortedV[i + 1];
            temp.right_vertex = sortedV[i];
            newNodes[count++] = temp;
        }
        return count;

    }

}