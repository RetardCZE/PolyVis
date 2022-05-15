#include "edgevis/search/visibility.h"

namespace edgevis{
    void
    expand(SearchNode node, const Mesh& mesh, std::vector<Point>& visibility, int level){

        if(node.next_polygon == -1){
            visibility.push_back(node.right);
            visibility.push_back(node.left);
            return;
        }
        int num;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::expand_searchnode(node, mesh, nodes);
        for(int i = 0; i < num; i++){

            expand(nodes[i], mesh, visibility, level+1);
        }
        delete [] nodes;
    }

    std::vector<Point>
    find_visibility( int edge, const Mesh& mesh, bool side){
        int num;
        std::vector<Point> r_vis;
        SearchNode* nodes = new edgevis::SearchNode[mesh.max_poly_sides + 2];
        num = edgevis::get_edge_init_nodes(mesh.mesh_edges[edge], side, mesh, nodes);
        for(int i = 0; i < num; i++){
            expand(nodes[i], mesh, r_vis, 0);
        }
        delete [] nodes;

        return r_vis;
    }
}