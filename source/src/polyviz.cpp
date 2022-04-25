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

std::vector<double>
PolyVis::read_measurements(){
    return this->measurements;
}

void PolyVis::expand_edge(polyanya::SearchNodePtr n, polyanya::Point root, int level){
    if(n->next_polygon == -1){
        this->vertices.push_back(n->right);
        this->vertices.push_back(n->left);
        return;
    }
    this->expansions++;
    polyanya::Successor* successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];
    /*
    Get all observable successors of the node. (Edges visible from root through edge)
    */

    int num_succ = polyanya::get_successors2(*n, root, *(this->mesh), successors);
    polyanya::SearchNode* nodes = new polyanya::SearchNode [num_succ];
    const int num_nodes = this->si->succ_to_node2(n, successors, num_succ, nodes);
    for(int i = 0; i < num_nodes; i++){
        n = &nodes[i];
        this->expand_edge(n, root, level + 1);
    }
    return;
}

std::vector<polyanya::Point>
PolyVis::get_visibility_polygon(polyanya::Point position){
    this->expansions = 0;
    this->vertices.clear();
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
        auto duration = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()) * 1e-9;
        this->measurements.push_back(duration);
        //std::cout << "Expanded edges " << expansions << " times." << std::endl;
    }
    delete si;
    return this->vertices;

}

double drand(const double min = 0., const double max = 1.)
{
    return (max - min) * static_cast<double>(rand()) / static_cast<double> (RAND_MAX)+min;
}

std::vector<polyanya::Point>
PolyVis::generate_points(int n){
    std::vector<polyanya::Point> points;
    polyanya::Point p;
    double min_x, max_x, min_y, max_y;
    this->mesh->get_bounding_box(&min_x, &max_x, &min_y, &max_y);

    for(; n > 0; n--)
    {
        p.x = drand(min_x, max_x);
        p.y = drand(min_y, max_y);
        polyanya::PointLocation loc = this->mesh->get_point_location(p);
        if(loc.type == polyanya::PointLocation::NOT_ON_MESH){
            n++;
        }else{
            points.push_back(p);
        }
    }
    return points;
}