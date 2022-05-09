
// === TRIVIS INCLUDES ===

#include "trivis/core/tri_vis.h"
#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/geom/generic_geom_types.h"
#include "trivis/core/utils/simple_clock.h"
#include "trivis/core/utils/clipper_utils.h"
#include "trivis/core/geom/generic_geom_utils.h"

#include "trivis/map_coverage/map_coverage.h"
#include "trivis/map_coverage/random_points.h"
#include "trivis/map_coverage/macs.h"

// === THIS PROJECT INCLUDES ===

#include "data_loading/load_map.h"
#include "drawing/drawing.h"
#include "drawing/random_colors.h"
#include "logging/logging.h"
#include "visualization.h"
#include "polyviz.h"
#include "edgevis/structs/mesh.h"
#include "edgevis//structs/edge.h"

#include "polyanya/parsers/map_parser.h"


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace tv = trivis;
namespace tvc = tv::core;
namespace tvg = tvc::geom;
namespace tve = trivis_examples;
namespace dr = tve::drawing;

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
    int calc = 0;
    for (Edge e : edgemesh.mesh_edges){
        if(e.rightPoly >= 0) {
            std::cout << e.parent << " | " << e.child << "\n" << e.leftPoly << " | " << e.rightPoly << "\n\n";
            calc++;
        }

    }
    std::cout << calc << std::endl;

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
