#include "polyanya/helpers/scenario.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/search/searchinstance.h"
#include "polyanya/structs/point.h"
#include "polyanya/search/expansion.h"
#include "geomMesh/parsers/map_parser.h"

#include "geom/geom.h"
#include "geom/colors.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>

/**
* https://stackoverflow.com/questions/20656129/generate-pointsx-y-on-rectangle-uniformly
*/

namespace cgm = cairo_geom_drawer;
namespace polyanya {
    class PolyVis {
        public:
            PolyVis(std::string file);

            PolyVis(parsers::GeomMesh &mesh);

            ~PolyVis();

            std::vector<Point> get_visibility_polygon(Point position);

            std::vector<Point> generate_points(int n);

            int expansions = 0;
            int max_depth = 0;
            SearchInstance *si;
            bool useRobustOrientation = true;

            void set_visual_mesh(const parsers::GeomMesh &gmesh);
            void visualise_segment(Point A, Point B, int color, float opacity);
            void visualise_long_segment(Point A, Point B, int color, float opacity);
            void visualise_point(Point A, int color, bool draw);
            void visualise_named_point(Point A, int color, std::string str);
            void visualise_polygon(std::vector<Point> &p, int color, bool draw, bool outline);
            void make_frame();
            void draw_base(bool boundaries);
            void reset_visu();
            std::string foldername = "imagesTri";

    private:
        void expand_edge(SearchNodePtr, Point root, int level);
        int frames = 0;
        std::vector<Point> vertices;

        Mesh *mesh;
        std::ifstream my_file;
        Successor *successors;

        cgm::CairoGeomDrawer cgm_drawer; // = cgm::CairoGeomDrawer(this->mesh->max_x, this->mesh->max_y, 20);
        parsers::GeomMesh gmesh;
        std::vector<geom::Polygon<double>> visu_polygons;
        Point seeker;
        Point currR, currL;
    };
}
