#include "polyviz.h"

PolyVis::PolyVis(std::string file){
    this->my_file = std::ifstream(file, std::ios::binary);
    this->mesh = new polyanya::Mesh(this->my_file);
    this->mesh->precalc_point_location();
}

PolyVis::~PolyVis(){
    this->my_file.close();
}

void PolyVis::expand_edge(polyanya::SearchNodePtr n, polyanya::Point root, int level){


    polyanya::Successor* successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];
    polyanya::Successor* obs_successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];

    /*
    Get all observable successors of the node. (Edges visible from root through edge)
    */
    int num_succ = polyanya::get_successors2(*n, root, *(this->mesh), successors);
    int num_obs_succ = 0;
    if(num_succ == 0){
        //std::cout << "no successors\n\n";
        this->vertices.push_back(this->mesh->mesh_vertices[n->right_vertex].p);
        this->vertices.push_back(this->mesh->mesh_vertices[n->left_vertex].p);
        return;
    }
    for(int i = 0; i < num_succ; i++){
        if(successors[i].type == polyanya::Successor::OBSERVABLE){
            std::string stuff(4*level, ' ');
            std::cout << stuff << successors[i] << "\n\n";
            obs_successors[num_obs_succ] = successors[i];
            num_obs_succ++;
        }
    }
    //std::cout << "_________________\n" <<num_obs_succ << "\n\n";
    if(num_obs_succ == 0){
    std::cout << "\nno successors\n";
    }
    /*
    Transforms successors to proper nodes
    */
    polyanya::SearchNode* nodes = new polyanya::SearchNode [num_obs_succ];
    const int num_nodes = this->si->succ_to_node2(n, successors, num_obs_succ, nodes);
    std::cout << num_nodes - num_obs_succ << "\n";
    for(int i = 0; i < num_nodes; i++){
        n = &nodes[i];
        std::string stuff(4*level+2, ' ');
        std::cout << stuff << *n <<"\n";
        //std::cout << num_nodes -i << " : " << *n <<"\n";
        this->expand_edge(n, root, level + 1);
    }
    return;
}

std::vector<polyanya::Point>
PolyVis::get_visibility_polygon(polyanya::Point position){
    si = new polyanya::SearchInstance(this->mesh);
    this->si->set_start_goal(position, position); //TODO: reimplement SearchInstance to get rid of useless parts
    std::vector<polyanya::SearchNodePtr> list = this->si->gen_initial_nodes2(); // custom version - minor updates

    polyanya::SearchNodePtr n;
    while(!list.empty()){
        n = list.front();
        std::cout << "\n-------------------------------------------------------------------\n\n";std::cout << *n <<"\n";
        list.erase(list.begin());
        this->expand_edge(n, position, 0);
    }
    return this->vertices;

}