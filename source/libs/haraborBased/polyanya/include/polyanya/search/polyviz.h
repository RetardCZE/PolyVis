#include "polyanya/helpers/scenario.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/search/searchinstance.h"
#include "polyanya/structs/point.h"
#include "polyanya/search/expansion.h"
#include "geomMesh/parsers/map_parser.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>

/**
* https://stackoverflow.com/questions/20656129/generate-pointsx-y-on-rectangle-uniformly
*/

class PolyVis{
    public:
        PolyVis(std::string file);
        PolyVis(parsers::GeomMesh &mesh);
        ~PolyVis();
        std::vector<polyanya::Point> get_visibility_polygon(polyanya::Point position);
        std::vector<polyanya::Point> generate_points(int n);

    private:
        void expand_edge(polyanya::SearchNodePtr, polyanya::Point root, int level);
        int expansions = 0;
        std::vector<polyanya::Point> vertices;
        polyanya::SearchInstance* si;
        polyanya::Mesh* mesh;
        std::ifstream my_file;
        polyanya::Successor* successors;
};
