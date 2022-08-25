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
#include "logging/logging.h"

#include "drawing/random_colors.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/search/visibility_utils.h"
#include "edgevis/search/visibility.h"

#include "geom/geom.h"
#include "geom/utils.h"
#include "geom/cairo_geom_drawer.h"

#include "polyanya/parsers/map_parser.h"

#include "polyviz.h"
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
namespace cgm = cairo_geom_drawer;

/*
 * Argument parsing taken from trivis_examples.cc
*/

struct ProgramOptionVariables {
    std::string input_map_name = "undefined";
    std::string input_map_extension = ".txt";
    std::string input_map_dir = INPUT_MAPS_DIR;
    std::string input_map_full_path;
    std::string mesh_type = "polygonal";
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

void local_visualise(parsers::GeomMesh &mesh, Edge& edge, std::vector<Point>& P, std::string name){
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
    //cgm_drawer.OpenPDF("edgevis_testing.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
    cgm_drawer.DrawPolygons(free, cgm::kColorWhite);
    cgm_drawer.DrawPoints(visibility, 0.2, cgm::kColorGreen);
    cgm_drawer.DrawPolygon(visibility, cgm::kColorLightGreen, 0.5);
    vertex = vertices[edge.parent];
    vertex2 = vertices[edge.child];
    cgm_drawer.DrawLine(vertex, vertex2, 0.1, cgm::kColorRed, 0.5);
    cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorBlue, 0.5);
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


    // Create and initialize TriVis object.
    tvc::TriVis vis;
    {   // Load map from file and move it to TriVis (without copying).
        tvg::PolyMap map;
        std::string load_msg = tve::data_loading::LoadPolyMapSafely(pov.input_map_full_path, map, pov.map_scale);
        if (load_msg != "ok") {
            LOGF_FTL("Error while loading map. " << load_msg);
            return EXIT_FAILURE;
        }
        map.RemoveDuplicatePoints();
        map.RemoveCollinearPoints();
        vis.SetMap(std::move(map));
        // cannot use map anymore
    }
    vis.TriangulateMapConstrainedDelaunay();
    vis.FillBucketing();
    vis.OptimizeBuckets();

    // Generate n random points in the map.
    std::vector<double> triangle_accum_areas; // auxiliary structure to improve speed
    std::mt19937 rng(pov.random_seed); // random generator
    tvg::FPoints random_points(pov.n_random_samples);

    std::vector<edgevis::Point> verticesTri;
    std::vector<edgevis::Point> verticesPoly;
    std::vector<edgevis::Point> r_points;
    std::vector<polyanya::Point> p_points;
    edgevis::Point p;
    polyanya::Point anyaP;
    std::optional<double> vis_radius = pov.vis_radius > 0.0 ? std::make_optional(pov.vis_radius) : std::nullopt;
    std::cout << "Generating points. (TriVis feature, points are converted for polyanya)\n";
    for (auto &rp: random_points) {
        rp = tv::map_coverage::UniformRandomPointInRandomTriangle(vis.triangles(), triangle_accum_areas, rng);
        p.x = rp.x;
        p.y = rp.y;
        anyaP.x = rp.x;
        anyaP.y = rp.y;
        r_points.push_back(p);
        p_points.push_back(anyaP);
    }

    std::vector<Point> r_v;
    std::vector<Point> l_v;
    std::vector<Point> v;

    std::string name;

    parsers::GeomMesh geomMesh;
    std::cout << pov.mesh_type << std::endl;
    if(pov.mesh_type == "polygonal"){
        geomMesh = geomMeshPoly;
    }else if(pov.mesh_type == "triangular"){
        geomMesh = geomMeshTri;
    }else{
        std::cout << "Unknown mesh type.\n";
        return -1;
    }

    edgevis::Mesh edgemesh;
    edgemesh.read(geomMesh);
    edgemesh.precalc_point_location();
    edgemesh.calculate_edges();

    PolyVis solverPoly(geomMesh);
    edgevis::EdgeVisibility Evis(edgemesh);
    Evis.set_visual_mesh(geomMesh);
    std::cout << edgemesh.is_convex();

    std::vector<polyanya::Point> verticesPolyAnya;
    std::cout << "Precomputing visibility of edges.\n";
    Evis.switch_debug(false);
    double time;
    tvc::utils::SimpleClock clock;

    clock.Restart();
    Evis.precompute_edges_searchnodes();
    return 1;
    /*
    time = clock.TimeInSeconds();
    std::cout << "\nEdgevis\n";
    std::cout << "Preprocessing time for EdgeVis was " <<
              time << " seconds.\n";

    clock.Restart();
    bool debug = pov.debug;
    bool save = pov.save;

    std::ofstream outfile;
    std::ofstream outfile_polyvis;
    std::ofstream results;
    results.open("results.dat", std::ios::out | std::ios::trunc );
    if(save){
        outfile.open("logger_edgevis.dat", std::ios::out | std::ios::trunc );
        outfile_polyvis.open("logger_polyvis.dat", std::ios::out | std::ios::trunc );
    }
    for (auto pos : r_points){
        if(debug)
            Evis.reset_visu();

        verticesPoly = Evis.find_point_visibility(pos, verticesPoly, debug);
        if(save){
            outfile << "--\n";
            outfile << pos << std::endl;
            outfile << verticesPoly.size() << std::endl;
            for (auto p : verticesPoly){
                outfile << p << "; ";
            }
            outfile << std::endl;
        }

        if(debug) {
            anyaP.x = pos.x;
            anyaP.y = pos.y;
            verticesPolyAnya = solverPoly.get_visibility_polygon(anyaP);

            Evis.visualise_point(pos, 0);
            Evis.visualise_polygon(verticesPoly, 2);
            verticesPoly.resize(verticesPolyAnya.size());
            for (int i = 0; i < verticesPolyAnya.size(); i++) {
                verticesPoly[i].x = verticesPolyAnya[i].x;
                verticesPoly[i].y = verticesPolyAnya[i].y;
            }
            Evis.visualise_polygon(verticesPoly, 1);
            getchar();
            system("clear");
        }

    }
    time = clock.TimeInSeconds();
    std::cout << "\nEdgevis\n";
    std::cout << "Total computation time of "<< pov.n_random_samples << " random points was " <<
              time << " seconds.\n";
    std::cout << "Mean computation time of "<< pov.n_random_samples << " random points was " <<
              time/pov.n_random_samples << " seconds/point.\n";
    results << "EdgeVis:";
    results << pov.input_map_name <<
              "\nNumber of iterations: " <<  pov.n_random_samples <<
              "\nTotal computation time:" << time <<
              "\nAverage computation time per point:" << time/pov.n_random_samples << std::endl;

    clock.Restart();
    for (auto pos : p_points){
        verticesPolyAnya = solverPoly.get_visibility_polygon(pos);
        if(save){
            outfile_polyvis << "--\n";
            outfile_polyvis << pos << std::endl;
            outfile_polyvis << verticesPolyAnya.size() << std::endl;
            for (auto p : verticesPolyAnya){
                outfile_polyvis << p << "; ";
            }
            outfile_polyvis << std::endl;

        }
    }
    time = clock.TimeInSeconds();
    std::cout << "\nPolyVis\n";
    std::cout << "Total computation time of "<< pov.n_random_samples << " random points was " <<
              time << " seconds.\n";
    std::cout << "Mean computation time of "<< pov.n_random_samples << " random points was " <<
              time/pov.n_random_samples << " seconds/point.\n";

    results << "\n===============================================================\n";
    results << "PolyVis:";
    results << pov.input_map_name <<
            "\nNumber of iterations: " <<  pov.n_random_samples <<
            "\nTotal computation time:" << time <<
            "\nAverage computation time per point:" << time/pov.n_random_samples << std::endl;

    return 0;
        */
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
