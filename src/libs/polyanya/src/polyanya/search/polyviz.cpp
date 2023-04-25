#include "polyanya/search/polyviz.h"
using namespace polyanya;
PolyVis::PolyVis(std::string file){
    this->my_file = std::ifstream(file, std::ios::binary);
    this->mesh = new Mesh(this->my_file);
    this->mesh->precalc_point_location();
    si = new SearchInstance(this->mesh);
    successors = new Successor [this->mesh->max_poly_sides + 2];
}

PolyVis::PolyVis(parsers::GeomMesh &mesh){
    this->mesh = new Mesh;
    this->mesh->read(mesh);
    this->mesh->precalc_point_location();
    si = new SearchInstance(this->mesh);
    successors = new Successor [this->mesh->max_poly_sides + 2];
}

PolyVis::~PolyVis(){
    this->my_file.close();
    delete si, successors;
}

void PolyVis::expand_edge(SearchNodePtr n, Point root, int level){
    if(n->next_polygon == -1){
        this->vertices.push_back(n->right);
        this->vertices.push_back(n->left);
        if(level > max_depth) max_depth = level;
        return;
    }
    this->expansions++;

    int num_succ = expand(*n, root, *(this->mesh), successors, useRobustOrientation);

    /*
     * Very important to alloc locally in function. Pointer to a element is passed in recursion, so
     * for each recursion level nodes needs to be kept separately!!!
     */
    SearchNode* nodes = new SearchNode [num_succ];
    const int num_nodes = this->si->successors2nodes(n, successors, num_succ, nodes);

    for(int i = 0; i < num_nodes; i++){
        n = &nodes[i];
        this->expand_edge(n, root, level + 1);
    }
    delete nodes;
    return;
}

std::vector<Point>
PolyVis::get_visibility_polygon(Point position){
    this->expansions = 0;
    this->max_depth = 0;
    this->vertices.clear();
    std::vector<SearchNodePtr> list = this->si->getInitNodes(position); // custom version - minor updates
    for(auto n : list){
        this->expand_edge(n, position, 0);
    }
    return this->vertices;
}

double drand(const double min = 0., const double max = 1.)
{
    return (max - min) * static_cast<double>(rand()) / static_cast<double> (RAND_MAX)+min;
}

std::vector<Point>
PolyVis::generate_points(int n){
    std::vector<Point> points;
    Point p;
    double min_x, max_x, min_y, max_y;
    this->mesh->get_bounding_box(&min_x, &max_x, &min_y, &max_y);

    for(; n > 0; n--)
    {
        p.x = drand(min_x, max_x);
        p.y = drand(min_y, max_y);
        PointLocation loc = this->mesh->get_point_location(p);
        if(loc.type == PointLocation::NOT_ON_MESH){
            n++;
        }else{
            points.push_back(p);
        }
    }
    return points;
}