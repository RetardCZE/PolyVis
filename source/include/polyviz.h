#include "polyanya/helpers/scenario.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/search/searchinstance.h"
#include "polyanya/structs/point.h"
#include "polyanya/search/expansion.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>


class PolyVis{
    public:
        PolyVis(std::string file);
        ~PolyVis();
        std::vector<polyanya::Point> get_visibility_polygon(polyanya::Point position);
    private:
        void expand_edge(polyanya::SearchNodePtr, polyanya::Point root);

        std::vector<polyanya::Point> vertices;
        polyanya::SearchInstance* si;
        polyanya::Mesh* mesh;
        std::ifstream my_file;
};
