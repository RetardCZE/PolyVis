#include "edgevis/structs/searchnode.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/point.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/structs/edge.h"
#include <vector>

namespace edgevis
{
    std::vector<int>
    normalise_ids(std::vector<int> elements, int new_start){
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
        return normalised;
    }

    int
    get_edge_init_nodes(Edge edge, bool side, const Mesh& mesh, SearchNode* initNodes){
        // side - true = right, false = left
        Polygon expander;
        SearchNode temp;
        Point A, B;
        int count = 0;
        std::vector<int> sortedV, sortedP;
        if(side) {
            if(edge.rightPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.rightPoly];
            sortedP = normalise_ids(expander.polygons, edge.leftPoly);
            sortedV = normalise_ids(expander.vertices, edge.parent);
        }else{
            if(edge.leftPoly == -1) return 0;
            expander = mesh.mesh_polygons[edge.leftPoly];
            sortedP = normalise_ids(expander.polygons, edge.rightPoly);
            sortedV = normalise_ids(expander.vertices, edge.child);
        }
        temp.parent = mesh.mesh_vertices[edge.parent].p;
        temp.child = mesh.mesh_vertices[edge.child].p;


        for(int i = 0; i < sortedV.size()-1; i++){
            if(sortedP[i+1] == -1) continue;
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

}