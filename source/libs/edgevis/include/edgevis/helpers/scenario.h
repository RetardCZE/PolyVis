#pragma once
#include "edgevis/structs/point.h"
#include <iostream>
#include <vector>
#include <string>

namespace edgevis
{

struct Scenario
{
    int bucket;
    double xsize, ysize;
    Point start, goal;
    double gridcost;
};

bool load_scenarios(std::istream& infile, std::vector<Scenario>& out);

}
