#include "edgevis/helpers/scenario.h"
#include "edgevis/structs/point.h"
#include <iostream>
#include <vector>
#include <string>

namespace edgevis
{

bool load_scenarios(std::istream& infile, std::vector<Scenario>& out)
{
    std::string version_str;
    double version;
    if (!(infile >> version_str >> version))
    {
        std::cerr << "Couldn't find scenario version." << std::endl;
        return false;
    }
    if (version_str != "version" || version != 1.0)
    {
        std::cerr << "Invalid scenario version." << std::endl;
        return false;
    }
    Scenario s;
    std::string map;
    while (infile >> s.bucket >> map >> s.xsize >> s.ysize >> s.start.x >>
           s.start.y >> s.goal.x >> s.goal.y >> s.gridcost)
    {
        out.push_back(s);
    }
    return true;
}

}
