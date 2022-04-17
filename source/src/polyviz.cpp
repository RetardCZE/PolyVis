#include "polyviz.h"

PolyVis::PolyVis(std::string file){
    this->my_file = std::ifstream(file, std::ios::binary);
    this->mesh = new polyanya::Mesh(this->my_file);
    this->mesh->precalc_point_location();
}

PolyVis::PolyVis(parsers::GeomMesh &mesh){
    this->mesh = new polyanya::Mesh;
    this->mesh->read(mesh);
    this->mesh->precalc_point_location();
}

PolyVis::~PolyVis(){
    this->my_file.close();
}

bool
PolyVis::switch_measurement(bool on, bool reset){
    this->measure = on;
    if(reset)this->measurements.clear();
    return this->measure;
}

std::vector<int>
PolyVis::read_measurements(){
    return this->measurements;
}

void PolyVis::expand_edge(polyanya::SearchNodePtr n, polyanya::Point root, int level){

    if(n->next_polygon == -1){
        this->vertices.push_back(n->right);
        this->vertices.push_back(n->left);
        return;
    }

    polyanya::Successor* successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];
    polyanya::Successor* obs_successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];

    /*
    Get all observable successors of the node. (Edges visible from root through edge)
    */
    int num_succ = polyanya::get_successors2(*n, root, *(this->mesh), successors);
    int num_obs_succ = 0;
    if(num_succ == 0){
        this->vertices.push_back(n->right);
        this->vertices.push_back(n->left);
        return;
    }
    for(int i = 0; i < num_succ; i++){
        if(successors[i].type == polyanya::Successor::OBSERVABLE){
            obs_successors[num_obs_succ] = successors[i];
            num_obs_succ++;
        }
    }

    /*
    Transforms successors to proper nodes
    */
    polyanya::SearchNode* nodes = new polyanya::SearchNode [num_obs_succ];
    const int num_nodes = this->si->succ_to_node2(n, obs_successors, num_obs_succ, nodes);
    for(int i = 0; i < num_nodes; i++){
        n = &nodes[i];
        std::string stuff(4*level+2, ' ');

        this->expand_edge(n, root, level + 1);
    }
    return;
}

std::vector<polyanya::Point>
PolyVis::get_visibility_polygon(polyanya::Point position){
    if(this->measure){
        start = std::chrono::high_resolution_clock::now();
    }

    si = new polyanya::SearchInstance(this->mesh);
    this->si->set_start_goal(position, position); //TODO: reimplement SearchInstance to get rid of useless parts
    std::vector<polyanya::SearchNodePtr> list = this->si->gen_initial_nodes2(); // custom version - minor updates

    polyanya::SearchNodePtr n;
    while(!list.empty()){
        n = list.front();
        list.erase(list.begin());
        this->expand_edge(n, position, 0);
    }
    if(this->measure){
        stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        this->measurements.push_back(duration.count());
    }
    return this->vertices;

}