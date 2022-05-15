
// === THIS PROJECT INCLUDES ===

#include "drawing/random_colors.h"
#include "visualization.h"
#include "polyviz.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/search/expansion.h"
#include "edgevis/search/visibility.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

#include "polyanya/parsers/map_parser.h"


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;



/*
 * Argument parsing taken from trivis_examples.cc
*/

struct ProgramOptionVariables {
    std::string input_map_name = "undefined";
    std::string input_map_extension = ".txt";
    std::string input_map_dir = INPUT_MAPS_DIR;
    std::string input_map_full_path;
    double map_scale = -1.0;
    double vis_radius = -1.0;
};

void AddProgramOptions(
        po::options_description &options_description,
        ProgramOptionVariables &pov
) {
    options_description.add_options()
            ("help,h", "Produce this help message. \n (*) Overwrites options: all.")
            ("map-name,m",
             po::value(&pov.input_map_name)->default_value(pov.input_map_name),
             "Map name.")
            ("map-ext",
             po::value(&pov.input_map_extension)->default_value(pov.input_map_extension),
             "Map file extension.")
            ("map-dir",
             po::value(&pov.input_map_dir)->default_value(pov.input_map_dir),
             "Map file directory.")
            ("map",
             po::value(&pov.input_map_full_path)->default_value(pov.input_map_full_path),
             "Full path to the map file. \n (*) Overwrites options: map_name, map_ext, map_dir.")
            ("map-scale,s",
             po::value(&pov.map_scale),
             "Map scale (optional).")
            ("vis-radius,r",
             po::value(&pov.vis_radius)->default_value(pov.vis_radius),
             "Visibility radius (-1 ~ infinite).");
}

char ParseProgramOptions(
        int argc,
        const char *const *argv,
        ProgramOptionVariables &pov
) {
    po::variables_map vm;
    po::options_description command_line_options;
    po::options_description options_description("General options");
    AddProgramOptions(options_description, pov);
    try {
        // Parse the command line arguments.
        command_line_options.add(options_description);
        po::store(po::parse_command_line(argc, argv, command_line_options), vm);
        po::notify(vm);
    } catch (const std::exception &e) {
        return 'e';
    }
    if (vm.count("help")) {
        // If '-h' or '--help' option, print the options and return 'h'.
        command_line_options.print(std::cout, 80);
        return 'h';
    }
    // Make input_file_name, input_file_extension, input_file_dir, and input_file_full_path consistent.
    if (pov.input_map_full_path.empty()) {
        pov.input_map_full_path = pov.input_map_dir + "/" + pov.input_map_name + (pov.input_map_extension.empty() ? "" : pov.input_map_extension);
    } else {
        auto aux = fs::path(pov.input_map_full_path);
        pov.input_map_name = fs::change_extension(aux, "").filename().string();
        pov.input_map_extension = aux.extension().string();
        pov.input_map_dir = aux.parent_path().string();
    }
    return '0';
}

void visualise(parsers::GeomMesh &mesh, Edge& edge, std::vector<Point>& P, std::string name){
    geom::Polygons<double> free;
    geom::Points<double> vertices;
    geom::Polygon<double> visibility;
    geom::Point<double> vertex, vertex2;
    geom::Polygon<double> polygon;
    for(auto v : mesh.vertices){
        vertices.push_back(v.point);
    }
    for(auto p : mesh.polygons){
        free.push_back(p.polygon);
    }
    for(auto v : P){
        vertex.x = v.x;
        vertex.y = v.y;
        visibility.push_back(vertex);
    }
    double x_min, x_max, y_min, y_max;
    geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
    geom::Polygon<double> border = {
            {x_min, y_min},
            {x_min, y_max},
            {x_max, y_max},
            {x_max, y_min},
    };
    cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);

    /// Create and save to PDF file
    cgm_drawer.OpenPDF(name);
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
    cgm_drawer.DrawPolygons(free, cgm::kColorWhite);
    cgm_drawer.DrawPoints(visibility, 0.2, cgm::kColorGreen);
    cgm_drawer.DrawPolygon(visibility, cgm::kColorLightGreen, 0.5);
    vertex = vertices[edge.parent];
    vertex2 = vertices[edge.child];
    cgm_drawer.DrawLine(vertex, vertex2, 0.2, cgm::kColorRed);
    cgm_drawer.Close();
    return;

}

int body(ProgramOptionVariables pov)
{
    std::cout << "Preparing meshes, initializing TriVis and PolyVis solvers.\n";
    parsers::MapParser mapParser;
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh geomMeshTri;
    parsers::GeomMesh geomMeshPoly;

    std::string mapName = pov.input_map_full_path;

    mapParser.convertMapToFade2DMesh(mapName, fade2DMesh);
    mapParser.convertMapToMergedMesh(mapName, mergedMesh);
    mapParser.convertMergedMeshToGeomMesh(mergedMesh, geomMeshPoly);
    mapParser.convertFade2DMeshToGeomMesh(fade2DMesh, geomMeshTri);
    edgevis::Mesh edgemesh;
    edgemesh.read(geomMeshPoly);
    edgemesh.calculate_edges();
    std::vector<Point> r_v;
    std::vector<Point> l_v;
    std::vector<Point> v;
    int c = 0;
    int spaceEdge;
    std::string name;
    for (Edge e : edgemesh.mesh_edges){
        if(c % 1 == 0) {
            name = "images/" + pov.input_map_name + "_" + std::to_string(c) + ".pdf";
            std::cout << name << std::endl ;
            spaceEdge = c;
            r_v.clear(); l_v.clear(); v.clear();
            r_v = edgevis::find_visibility(spaceEdge, edgemesh, true);
            l_v = edgevis::find_visibility(spaceEdge, edgemesh, false);
            v.reserve( r_v.size() + l_v.size() ); // preallocate memory
            v.insert( v.end(), r_v.begin(), r_v.end() );
            v.insert( v.end(), l_v.begin(), l_v.end() );
            visualise(geomMeshPoly, edgemesh.mesh_edges[spaceEdge], v, name);
        }
        c++;

    }
    return 0;
}

int main(
        int argc,
        const char *const *argv
) {
    ProgramOptionVariables pov;
    char c = ParseProgramOptions(argc, argv, pov);
    if (c == 'h') {
        return EXIT_SUCCESS;
    } else if (c == 'e') {
        return EXIT_FAILURE;
    } else {
        return body(pov);
    }
}
