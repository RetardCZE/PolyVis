#pragma once

#include <fade2d/Fade_2D.h>
#include <cstdlib>
#include <cstdio>
#include <algorithm>

namespace fadeutils {

    using namespace std;
    using namespace GEOM_FADE2D;

    typedef vector<Point2> Polygon;

    void fail(const string &message);

    void read_polys(istream &infile, vector<Polygon> &polygons);

    void load_map(const std::string &filename, vector<Polygon> &polygons);

    vector<ConstraintGraph2 *> *create_constraint_graphs(const vector<Polygon> &polygons, Fade_2D &dt);

    Zone2 *create_traversable_zone_istream(istream &infile, Fade_2D &dt, vector<Polygon> &obstacles);

    Zone2 *create_traversable_zone(const vector<Polygon> &polygons, Fade_2D &dt);

    Zone2 *create_traversable_zone_filename(const std::string &filename, Fade_2D &dt, vector<Polygon> &obstacles);

}
