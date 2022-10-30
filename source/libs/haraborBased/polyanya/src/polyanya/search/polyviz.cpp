#include "polyanya/search/polyviz.h"

PolyVis::PolyVis(std::string file){
    this->my_file = std::ifstream(file, std::ios::binary);
    this->mesh = new polyanya::Mesh(this->my_file);
    this->mesh->precalc_point_location();
    si = new polyanya::SearchInstance(this->mesh);
    successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];
}

PolyVis::PolyVis(parsers::GeomMesh &mesh){
    this->mesh = new polyanya::Mesh;
    this->mesh->read(mesh);
    this->mesh->precalc_point_location();
    si = new polyanya::SearchInstance(this->mesh);
    successors = new polyanya::Successor [this->mesh->max_poly_sides + 2];
}

PolyVis::~PolyVis(){
    this->my_file.close();
    delete si, successors;
}

void PolyVis::expand_edge(polyanya::SearchNodePtr n, polyanya::Point root, int level){
    if(n->next_polygon == -1){
        this->vertices.push_back(n->right);
        this->vertices.push_back(n->left);
        return;
    }
    this->expansions++;

    int num_succ = polyanya::expand(*n, root, *(this->mesh), successors);

    /*
     * Very important to alloc locally in function. Pointer to a element is passed in recursion, so
     * for each recursion level nodes needs to be kept separately!!!
     */
    polyanya::SearchNode* nodes = new polyanya::SearchNode [num_succ];
    const int num_nodes = this->si->successors2nodes(n, successors, num_succ, nodes);
    for(int i = 0; i < num_nodes; i++){
        n = &nodes[i];
        this->expand_edge(n, root, level + 1);
    }
    delete nodes;
    return;
}

std::vector<polyanya::Point>
PolyVis::get_visibility_polygon(polyanya::Point position){
    this->expansions = 0;
    this->vertices.clear();
    std::vector<polyanya::SearchNodePtr> list = this->si->getInitNodes(position); // custom version - minor updates

    polyanya::SearchNodePtr n;
    while(!list.empty()){
        n = list.front();
        list.erase(list.begin());
        this->expand_edge(n, position, 0);
    }
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