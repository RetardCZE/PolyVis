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
        bool a = (L == Orientation::CW || L == Orientation::COLLINEAR) && (p != left_child);
        bool b = (R == Orientation::CCW || R == Orientation::COLLINEAR) && (p != right_child);
        return a && b;
    }

    std::vector<int>
    normalise_ids(std::vector<int> elements, int new_start, int& shift){
        std::vector<int> normalised;
        int S = elements.size();
        normalised.resize(S);
        int offset = 0;
        int current = elements[offset];

        while(current != new_start){
            current = elements[++offset];
        }

        for(int i = 0; i < S; i++){
            normalised[i] = elements[(i+offset) % S];
        }
        shift = offset;
        return normalised;
    }

    int
    get_edge_init_nodes(Edge edge, bool side, const Mesh& mesh, SearchNode* initNodes){
        // side - true = right, false = left
        Polygon expander;
        SearchNode temp;
        Point A, B;
        int count = 0;
        int offset;
        std::vector<int> sortedV, sortedP;
        if(side) {
            if(edge.rightPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.rightPoly];
            //sortedP = normalise_ids(expander.polygons, edge.leftPoly);
            sortedV = normalise_ids(expander.vertices, edge.parent, offset);
            int S = sortedV.size();
            sortedP.resize(S);
            for(int i = 0; i < S; i++){
                sortedP[i] = expander.polygons[(i+offset) % S];
            }
            temp.parent = mesh.mesh_vertices[edge.parent].p;
            temp.child = mesh.mesh_vertices[edge.child].p;
            temp.coming_from = edge.rightPoly;
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.leftPoly];
            //sortedP = normalise_ids(expander.polygons, edge.rightPoly);
            sortedV = normalise_ids(expander.vertices, edge.child, offset);
            int S = sortedV.size();
            sortedP.resize(S);
            for(int i = 0; i < S; i++){
                sortedP[i] = expander.polygons[(i+offset) % S];
            }
            temp.parent = mesh.mesh_vertices[edge.child].p;
            temp.child = mesh.mesh_vertices[edge.parent].p;
            temp.coming_from = edge.leftPoly;
        }

        for(int i = 0; i < sortedV.size()-1; i++){
            A = mesh.mesh_vertices[sortedV[i]].p;
            B = mesh.mesh_vertices[sortedV[i+1]].p;
            temp.right = A;
            temp.left = B;
            temp.right_vertex = sortedV[i];
            temp.left_vertex = sortedV[i+1];
            temp.next_polygon = sortedP[i+1];
            initNodes[count++] = temp;
        }

        return count;
    }

    void
    recompute_roots(SearchNode* nodes, SearchNode parent, int num){
        Point parent_intersection, child_intersection;
        bool P, C;
        for(int i = 0; i < num; i++){
            P = is_observable(nodes[i].left,
                              parent.right,
                              nodes[i].right,
                              parent.left,
                              nodes[i].parent);
            C = is_observable(nodes[i].left,
                              parent.right,
                              nodes[i].right,
                              parent.left,
                              nodes[i].child);

            parent_intersection = line_intersect(nodes[i].left,
                                                 parent.right,
                                                 nodes[i].child,
                                                 nodes[i].parent);
            child_intersection = line_intersect(nodes[i].right,
                                                parent.left,
                                                nodes[i].parent,
                                                nodes[i].child);

            if(!P) nodes[i].parent = parent_intersection;
            if(!C) nodes[i].parent = child_intersection;
        }
    }

    int
    expand_searchnode(SearchNode node, const Mesh& mesh, SearchNode* newNodes){
        SearchNode temp;
        temp.coming_from = node.next_polygon;
        temp.parent = node.parent;
        temp.child = node.child;
        // TODO: prove it can be just assigned like this
        // right line
        Point right_child = node.right;
        Point right_parent = node.child;

        // left line
        Point left_child = node.left;
        Point left_parent = node.parent;

        const Polygon& expander = mesh.mesh_polygons[node.next_polygon];
        std::vector<int> sortedV, sortedP;

        int offset;
        //sortedP = normalise_ids(expander.polygons, node.coming_from);
        sortedV = normalise_ids(expander.vertices, node.right_vertex, offset);
        const int S = sortedV.size();
        sortedP.resize(S);
        for(int i = 0; i < S; i++){
            sortedP[i] = expander.polygons[(i+offset) % S];
        }

        int i = -1;
        int right_visible = S - 1;
        int left_visible = S - 1;
        while(i < S-2){
            if (is_observable(left_parent, left_child, right_parent, right_child, mesh.mesh_vertices[sortedV[++i]].p)){
                right_visible = i;
                break;
            }
        }
        i = S;
        while(i > 0){
            if (is_observable(left_parent, left_child, right_parent, right_child, mesh.mesh_vertices[sortedV[--i]].p)){
                left_visible = i;
                break;
            }
        }
        Point right_intersection, left_intersection;
        int count = 0;
        i = right_visible;

        if(right_visible > 0){
            right_intersection = line_intersect(mesh.mesh_vertices[sortedV[right_visible-1]].p,
                                                mesh.mesh_vertices[sortedV[right_visible]].p,
                                                right_parent, right_child);

            temp.next_polygon = sortedP[i];
            temp.left = mesh.mesh_vertices[sortedV[right_visible]].p;
            temp.right = right_intersection;
            temp.left_vertex = sortedV[right_visible];
            temp.right_vertex = sortedV[right_visible-1];
            newNodes[count++] = temp;
        }
        for(; i < left_visible; i++){
            temp.next_polygon = sortedP[i+1];
            temp.left = mesh.mesh_vertices[sortedV[i+1]].p;
            temp.right = mesh.mesh_vertices[sortedV[i]].p;
            temp.left_vertex = sortedV[i+1];
            temp.right_vertex = sortedV[i];
            newNodes[count++] = temp;
        }

        if(left_visible < S-1){
            left_intersection = line_intersect(mesh.mesh_vertices[sortedV[left_visible]].p,
                                                mesh.mesh_vertices[sortedV[left_visible+1]].p,
                                                left_parent, left_child);
            temp.next_polygon = sortedP[i+1];
            temp.left = left_intersection;
            temp.right = mesh.mesh_vertices[sortedV[i]].p;
            temp.left_vertex = sortedV[i+1];
            temp.right_vertex = sortedV[i];
            newNodes[count++] = temp;
        }
        recompute_roots(newNodes, node, count);
        return count;
    }

}