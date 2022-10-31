
// === THIS PROJECT INCLUDES ===
#include "data_loading/load_map.h"
#include "drawing/drawing.h"
#include "logging/logging.h"

#include "drawing/random_colors.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/search/expansion.h"
#include "edgevis/search/visibility.h"

#include "drawing/cairo_geom_drawer.h"

#include "geomMesh/parsers/map_parser.h"
#include "polyanya/search/polyviz.h"
#include "utils/simple_clock.h"

#include <iomanip>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace cgm = cairo_geom_drawer;

/*
 * Argument parsing taken from trivis_examples.cc
*/

struct ProgramOptionVariables {
    std::string input_map_name = "undefined";
    std::string input_map_extension = ".mesh";
    std::string input_map_dir = INPUT_MAPS_DIR;
    std::string input_map_full_path;
    std::string mesh_type = "triangular";
    bool debug = false;
    bool save = false;
    double map_scale = -1.0;
    double vis_radius = -1.0;
    int n_random_samples = 1;
    unsigned random_seed = std::random_device{}();
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
            ("mesh-type,t",
             po::value(&pov.mesh_type)->default_value(pov.mesh_type),
             "Type of used mesh - (triangular, polygonal) - default: polygonal.")
            ("vis-radius,r",
             po::value(&pov.vis_radius)->default_value(pov.vis_radius),
             "Visibility radius (-1 ~ infinite).")
            ("n-random-samples,n",
             po::value(&pov.n_random_samples)->default_value(pov.n_random_samples),
             "How many random samples should be generated.")
            ("random-seed",
             po::value(&pov.random_seed)->default_value(pov.random_seed),
             "Seed for the random generator (generated by std::random_device{}() by default).")
            ("debug",
             po::bool_switch(&pov.debug)->default_value(pov.debug),
             "Run in debug mode.")
            ("save",
             po::bool_switch(&pov.save)->default_value(pov.save),
             "Log resulting polygons to files (cannot measure performance)..");
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


int body(ProgramOptionVariables pov)
{
    std::cout << "Preparing meshes, initializing Edgevis, TriVis and PolyVis solvers.\n";

    parsers::MapParser mapParser;
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh geomMeshTri;
    parsers::GeomMesh geomMeshPoly;
    parsers::GeomMesh IronHarvest;

    std::string mapName = pov.input_map_full_path;

    /*
     * mapParser.convertMapToFade2DMesh(mapName, fade2DMesh);
     * mapParser.convertMapToMergedMesh(mapName, mergedMesh);
     * mapParser.convertMergedMeshToGeomMesh(mergedMesh, geomMeshPoly);
     * mapParser.convertFade2DMeshToGeomMesh(fade2DMesh, geomMeshTri);
     */
    mapParser.readGeomMeshFromIronHarvestMesh(mapName,
                                              IronHarvest);

    std::vector<edgevis::Point> verticesTri;
    std::vector<edgevis::Point> verticesPoly;
    std::vector<edgevis::Point> r_points;
    edgevis::Point p;

    std::vector<Point> r_v;
    std::vector<Point> l_v;
    std::vector<Point> v;

    std::string name;

    parsers::GeomMesh geomMesh;
    if(pov.mesh_type == "polygonal"){
        geomMesh = geomMeshPoly;
    }else if(pov.mesh_type == "triangular"){
        geomMesh = geomMeshTri;
    }else {
        std::cout << "Unknown mesh type.\n";
        return -1;
    }
    geomMesh = IronHarvest;

    edgevis::Mesh edgemesh;
    edgemesh.read(geomMesh);
    edgemesh.precalc_point_location();
    edgemesh.calculate_edges();

    edgevis::EdgeVisibility Evis(edgemesh);
    Evis.set_visual_mesh(geomMesh);
    Evis.switch_debug(false);

    Evis.precompute_edges_searchnodes();
    Evis.precompute_edges_optimnodesV1();

    Evis.reset_visu();
    std::vector<SearchNode> rVis, lVis;
    // 2542, 3595 |  2542, 2579 | 2579, 2712
    int parent = 2712;
    int child = 2579;
    Evis.find_arbitrary_edge_visibility(parent, child, lVis, rVis);
    Evis.visualise_segment(Evis.mesh_reference().mesh_vertices[parent].p,
                           Evis.mesh_reference().mesh_vertices[child].p,
                           0, 0.5);
    std::vector<Point> rPoints, lPoints;
    if(rVis.size() > 0){
        for(auto sn : rVis){
            rPoints.push_back(Evis.evaluate_intersection(sn.transitionR));
            rPoints.push_back(Evis.evaluate_intersection(sn.transitionL));
        }
        Evis.visualise_polygon(rPoints, 2);
    }
    if(lVis.size() > 0){
        for(auto sn : lVis){
            lPoints.push_back(Evis.evaluate_intersection(sn.transitionR));
            lPoints.push_back(Evis.evaluate_intersection(sn.transitionL));
        }
        Evis.visualise_polygon(lPoints, 1);
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
