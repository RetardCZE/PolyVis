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
            sortedV = normalise_ids(expander.vertices, edge.child, offset);
            int S = sortedV.size();
            sortedP.resize(S);
            for(int i = 0; i < S; i++){
                sortedP[i] = expander.polygons[(i+offset) % S];
            }
            temp.parent = mesh.mesh_vertices[edge.parent].p;
            temp.child = mesh.mesh_vertices[edge.child].p;
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

    bool
    is_on_abscissa(Point A, Point B, Point C){
        Point V = B-A;
        Point T = C-A;
        double dx, dy;
        dx = T.x / V.x;
        dy = T.y / V.y;
        if (EPSILON > std::abs(dy - dx) && dx <= 1 && dx >= 0 && dy <= 1 && dy >= 0)
            return true;
        return false;
    }

    void
    recompute_roots(SearchNode* nodes, SearchNode parent, int num){
        Point parent_intersection, child_intersection;
        bool P, C;
        for(int i = 0; i < num; i++){

            parent_intersection = line_intersect(nodes[i].left,
                                                 parent.right,
                                                 nodes[i].child,
                                                 nodes[i].parent);


            child_intersection = line_intersect(nodes[i].right,
                                                parent.left,
                                                nodes[i].parent,
                                                nodes[i].child);

            if(is_on_abscissa(nodes[i].parent, nodes[i].child, parent_intersection))
                parent_intersection == parent_intersection ? nodes[i].parent = parent_intersection : nodes[i].parent;

            if(is_on_abscissa(nodes[i].parent, nodes[i].child, child_intersection))
                child_intersection == child_intersection ? nodes[i].child = child_intersection : nodes[i].parent;
        }
    }

    int
    expand_searchnode(SearchNode node, const Mesh& mesh, SearchNode* newNodes){
        SearchNode temp;
        //std::cout << node.coming_from << " --> " <<node.next_polygon <<std::endl;
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

        int i;
        int right_visible = S - 1;
        int left_visible = 0;
        for(i = 0; i<S; i++){
            if (is_observable(right_child, left_child, right_parent, right_child, mesh.mesh_vertices[sortedV[i]].p)){
                right_visible = i;
                break;
            }
        }
        for(i = 0; i<S; i++){
            if (is_observable(left_parent, left_child, left_child, right_child, mesh.mesh_vertices[sortedV[S-1-i]].p)){
                left_visible = S-1-i;
                break;
            }
        }

        Point right_intersection, left_intersection;
        int count = 0;

        int visible = left_visible - right_visible;
        //std::cout << left_visible << " | " << right_visible << " | " << visible << std::endl;

        right_intersection = line_intersect(mesh.mesh_vertices[sortedV[right_visible - 1]].p,
                                            mesh.mesh_vertices[sortedV[right_visible]].p,
                                            right_parent, right_child);


        left_intersection = line_intersect(mesh.mesh_vertices[sortedV[left_visible]].p,
                                           mesh.mesh_vertices[sortedV[left_visible + 1]].p,
                                           left_parent, left_child);
        /*
        if(!(right_intersection==right_intersection)){
            std::cout << right_intersection << " | " << mesh.mesh_vertices[sortedV[right_visible - 1]].p << " | " <<
                    mesh.mesh_vertices[sortedV[right_visible]].p<< " | " <<
                    right_parent<< " | " << right_child << std::endl;
        }
        if(!(left_intersection==left_intersection)){
            std::cout << left_intersection << " | " << mesh.mesh_vertices[sortedV[left_visible]].p << " | " <<
                      mesh.mesh_vertices[sortedV[left_visible+1]].p<< " | " <<
                      left_parent<< " | " << left_child << std::endl;
        }*/
        if(visible < 0){
            temp.right = right_intersection;
            temp.left = left_intersection;
            temp.next_polygon = sortedP[right_visible];
            temp.left_vertex = sortedV[right_visible];
            temp.right_vertex = sortedV[left_visible];
            newNodes[count++] = temp;
        }else if(visible == 0){
            temp.right = right_intersection;
            temp.left = mesh.mesh_vertices[sortedV[right_visible]].p;
            temp.next_polygon = sortedP[right_visible];
            temp.left_vertex = sortedV[right_visible];
            temp.right_vertex = sortedV[right_visible-1];
            newNodes[count++] = temp;

            temp.right = mesh.mesh_vertices[sortedV[left_visible]].p;
            temp.left = left_intersection;
            temp.next_polygon = sortedP[left_visible+1];
            temp.left_vertex = sortedV[left_visible+1];
            temp.right_vertex = sortedV[left_visible];
            newNodes[count++] = temp;
        }else{
            temp.right = right_intersection;
            temp.left = mesh.mesh_vertices[sortedV[right_visible]].p;
            temp.next_polygon = sortedP[right_visible];
            temp.left_vertex = sortedV[right_visible];
            temp.right_vertex = sortedV[right_visible-1];
            newNodes[count++] = temp;

            for( i=0; i < visible; i++){
                temp.right = mesh.mesh_vertices[sortedV[right_visible+i]].p;
                temp.left = mesh.mesh_vertices[sortedV[right_visible+1+i]].p;
                temp.next_polygon = sortedP[right_visible + 1 + i];
                temp.left_vertex = sortedV[right_visible+i+1];
                temp.right_vertex = sortedV[right_visible+i];
                newNodes[count++] = temp;
            }

            temp.right = mesh.mesh_vertices[sortedV[left_visible]].p;
            temp.left = left_intersection;
            temp.next_polygon = sortedP[left_visible+1];
            temp.left_vertex = sortedV[left_visible+1];
            temp.right_vertex = sortedV[left_visible];
            newNodes[count++] = temp;
        }
        recompute_roots(newNodes, node, count);
        return count;

    }

}