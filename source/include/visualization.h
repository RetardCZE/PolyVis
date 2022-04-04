#include "draw_lib/geom/geom.h"
#include "draw_lib/geom/utils.h"
#include "draw_lib/cairo_geom_drawer/cairo_geom_drawer.h"
#include "draw_lib/polygon_clipping/polygon_clipping.h"
#include "draw_lib/polygonal_map/polygonal_map.h"

//include "polyanya"
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

namespace clip = polygon_clipping;
namespace cgm = cairo_geom_drawer;
namespace map = polygonal_map;
namespace draw = map::draw;
struct Line{
    geom::Point<double> left;
    geom::Point<double> right;
};
class MapVisualizer{
    enum Type
        {
            // Does not use any ints.
            NOT_ON_MESH,

            // Uses poly1 (the polygon it is on).
            IN_POLYGON,

            // Uses poly1 (the polygon it is on) and both vertices.
            ON_MESH_BORDER,       // edge: a polygon is not traversable

            // Uses poly1, poly2 and both vertices.
            ON_EDGE,              // edge: both polygons are traversable

            // Uses vertex1.
            // Can use poly1 to specify the "grid corrected poly".
            // Will need to manually assign poly1, though.
            ON_CORNER_VERTEX_AMBIG,   // vertex; two+ polygons are not traversable

            // Uses vertex1. Also returns an arbirary traversable adjacent
            // polygon in poly1.
            ON_CORNER_VERTEX_UNAMBIG, // vertex; one polygon is not traversable

            // Uses vertex1. Also returns an arbitrary adjacent polygon in poly1.
            ON_NON_CORNER_VERTEX, // vertex: all polygons are traversable
        };
    public:
        MapVisualizer(std::string file);
        ~MapVisualizer();

        void redraw();
        void parse_mesh();
        void set_visible_polygon(polyanya::Point p, std::vector<polyanya::Point> polygon);
    private:
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        geom::Polygon<double> visibility;
        geom::Point<double> seeker;
        std::ifstream my_file;

};
