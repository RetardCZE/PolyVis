#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/structs/edge.h"
#include "edgevis/helpers/geometry.h"
#include "edgevis/search/intersections.h"
#include "edgevis/search/visibility.h"
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
    recompute_roots(SearchNode &node, edgevis::EdgeVisibility *eObject) {
        Point parent_intersection, child_intersection, temp;
        float ratioPx, ratioPy, ratioCx, ratioCy;
        SearchNode* parent;
        parent = node.predecessor;
        while(parent){

            uint8_t parentIntersectionCheck = robust_geom::LineSegmentIntersectionGeneral(node.child_L,
                                                                                          parent->child_R,
                                                                                          node.root_R,
                                                                                          node.root_L,
                                                                                          parent_intersection);

            uint8_t childIntersectionCheck = robust_geom::LineSegmentIntersectionGeneral(node.child_R,
                                                                                         parent->child_L,
                                                                                         node.root_R,
                                                                                         node.root_L,
                                                                                         child_intersection);

            switch(parentIntersectionCheck){
                case 2:
                case 3:
                case 6:
                    break;
                case 0:
                case 5:
                    temp = parent->child_R - node.child_L;
                    ratioPx = temp.x / ((parent_intersection - node.child_L).x + 1e-12);
                    ratioPy = temp.y / ((parent_intersection - node.child_L).y + 1e-12);
                    if (ratioPx > 0 || ratioPy > 0)
                        node.root_R = parent_intersection;
                    break;
                case 1:
                    std::cerr << "Case 1\n";
                case 4:
                    std::cerr << "Case 4\n";
                default:
                    std::cout << node;
                    eObject->reset_visu();
                    eObject->visualise_segment(node.child_R, node.child_L, 0, 0.5);
                    eObject->visualise_segment(node.root_R, node.root_L, 1, 0.5);
                    eObject->visualise_segment(parent->child_R, parent->child_L, 2, 0.5);
                    std::cout << parent_intersection << " | " << static_cast<int>(parentIntersectionCheck) << std::endl;
                    std::cout << child_intersection << " | " << static_cast<int>(childIntersectionCheck) << std::endl;
                    eObject->visualise_point(parent_intersection, 0);
                    eObject->visualise_point(child_intersection, 2);
                    std::cerr << "Root R recomputation crashed." <<  std::endl;
                    eObject->visualise_point(node.root_L, 3);
                    eObject->visualise_point(node.root_R, 1);
                    getchar();break;
            }
            switch(childIntersectionCheck) {
                case 1:
                case 3:
                case 5:
                    break;
                case 0:
                case 6:
                    temp = parent->child_L - node.child_R;
                    ratioCx = temp.x / ((child_intersection - node.child_R).x + 1e-12);
                    ratioCy = temp.y / ((child_intersection - node.child_R).y + 1e-12);
                    if (ratioCy > 0 || ratioCx > 0)
                        node.root_L = child_intersection;
                    break;
                case 2:
                    std::cerr << "Case 2\n";
                case 4:
                    std::cerr << "Case 4\n";
                default:
                    std::cout << node;
                    eObject->reset_visu();
                    eObject->visualise_segment(node.child_R, node.child_L, 0, 0.5);
                    eObject->visualise_segment(node.root_R, node.root_L, 1, 0.5);
                    eObject->visualise_segment(parent->child_R, parent->child_L, 2, 0.5);
                    std::cout << parent_intersection << " | " << static_cast<int>(parentIntersectionCheck) << std::endl;
                    std::cout << child_intersection << " | " << static_cast<int>(childIntersectionCheck) << std::endl;
                    eObject->visualise_point(parent_intersection, 0);
                    eObject->visualise_point(child_intersection, 2);

                    eObject->visualise_point(node.root_L, 3);
                    eObject->visualise_point(node.root_R, 1);
                    getchar();
                    break;
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
            if (robust_geom::Orient(right_parent, right_child, mesh.mesh_vertices[sorted_vertices[S-1-i]].p) == robust_geom::Orientation::kRightTurn){
                if( i > 0)
                    break;
            }
            *right_visible = S-1-i;
        }
        for(i = 0; i<S; i++){
            if (robust_geom::Orient(left_parent, left_child, mesh.mesh_vertices[sorted_vertices[i]].p) == robust_geom::Orientation::kLeftTurn){
                if( i > 0)
                    break;
            }
            *left_visible = i;
        }
        return *left_visible - *right_visible;
    }

    int
    expand_searchnode(SearchNode &node, const Mesh &mesh, SearchNode *newNodes, EdgeVisibility *eObject) {
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
        uint8_t rCheck = 0;
        uint8_t lCheck = 0;
        int right_visible, left_visible;
        int visible = find_visible(mesh, node, sortedV, &right_visible, &left_visible);

        Point right_intersection, left_intersection;
        int count = 0;

        if (robust_geom::Orient(right_child, right_parent, mesh.mesh_vertices[sortedV[right_visible]].p) ==
            robust_geom::Orientation::kCollinear ||
            right_visible == 0) {
            right_intersection = mesh.mesh_vertices[sortedV[right_visible]].p;
        } else {
            rCheck = robust_geom::LineSegmentIntersectionGeneral(
                    right_parent, right_child, mesh.mesh_vertices[sortedV[right_visible - 1]].p,
                    mesh.mesh_vertices[sortedV[right_visible]].p, right_intersection);

            switch (rCheck) {
                case 0:
                    break;
                case 1:
                    right_intersection = mesh.mesh_vertices[sortedV[right_visible]].p;
                    break;
                default:
                    break;
            }
        }

        if (robust_geom::Orient(left_parent, left_child, mesh.mesh_vertices[sortedV[left_visible]].p) ==
            robust_geom::Orientation::kCollinear ||
            left_visible == S - 1) {
            left_intersection = mesh.mesh_vertices[sortedV[left_visible]].p;
        } else {
            lCheck = robust_geom::LineSegmentIntersectionGeneral(
                    left_parent, left_child, mesh.mesh_vertices[sortedV[left_visible]].p,
                    mesh.mesh_vertices[sortedV[left_visible + 1]].p, left_intersection);
            switch (lCheck) {
                case 0:
                    break;
                case 2:
                    left_intersection = mesh.mesh_vertices[sortedV[left_visible]].p;
                    break;
                default:
                    break;
            }
        }
        switch (lCheck) {
            default:
                std::cout << node;
                eObject->visualise_segment(right_parent, right_child, 0, 0.5);
                eObject->visualise_segment(left_parent, left_child, 1, 0.5);
                eObject->visualise_segment(node.child_R, node.child_L, 2, 0.5);
                std::cout << left_intersection << " | " << right_intersection << std::endl;
                std::cout << static_cast<int>(lCheck) << " | " << static_cast<int>(rCheck) << std::endl;
                std::cout << visible << " | " << S << " | " << left_visible << " | " << right_visible << std::endl;

                getchar();
                break;
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