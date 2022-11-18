// === THIS PROJECT INCLUDES ===

#include "data_loading/load_map.h"
#include "drawing/drawing.h"
#include "logging/logging.h"

#include "drawing/random_colors.h"
#include "edgevis/structs/mesh.h"
#include "edgevis/structs/edge.h"
#include "edgevis/structs/searchnode.h"
#include "edgevis/search/expansion.h"
#include "edgevis/search/edge_visibility.h"

#include "drawing/cairo_geom_drawer.h"

#include "geomMesh/parsers/map_parser.h"
#include "polyanya/search/polyviz.h"
#include "utils/simple_clock.h"

#include <iomanip>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <ncurses.h>
#include <opencv2/opencv.hpp>

#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif
#ifndef INPUT_CONVERTED_MAPS_DIR
#define INPUT_CONVERTED_MAPS_DIR "."
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
    std::string input_map_converted_dir = INPUT_CONVERTED_MAPS_DIR;
    std::string input_map_full_path;
    std::string input_map_converted_full_path;
    std::string mesh_type = "triangular";
    bool debug = false;
    bool save = false;
    bool heatmap = false;
    bool machine = true;
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
             "Log resulting polygons to files (cannot measure performance)..")
            ("heatmap",
             po::bool_switch(&pov.heatmap)->default_value(pov.heatmap),
             "Create heatmap of computation time")
            ("machine",
             po::bool_switch(&pov.machine)->default_value(pov.machine),
             "Save output in format dedicated for further SW processing.");
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
        pov.input_map_converted_full_path = pov.input_map_converted_dir + "/" + pov.input_map_name + ".txt";
    } else {
        auto aux = fs::path(pov.input_map_full_path);
        pov.input_map_name = fs::change_extension(aux, "").filename().string();
        pov.input_map_extension = aux.extension().string();
        pov.input_map_dir = aux.parent_path().string();
    }
    return '0';
}

int body(ProgramOptionVariables pov) {
    double time;
    custom::utils::SimpleClock clock;

    std::cout << "Preparing meshes, initializing Edgevis, TriVis and PolyVis solvers.\n";
    parsers::MapParser mapParser;
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh geomMeshTri;
    parsers::GeomMesh geomMeshPoly;
    parsers::GeomMesh IronHarvest;

    std::string mapName = pov.input_map_full_path;
    std::string convertedMapName = pov.input_map_converted_full_path;

    mapParser.convertMapToMergedMesh(convertedMapName, mergedMesh);
    mapParser.convertMergedMeshToGeomMesh(mergedMesh, geomMeshPoly);

    mapParser.readGeomMeshFromIronHarvestMesh(mapName,
                                              IronHarvest);
    pov.mesh_type = "triangular";

    std::mt19937 rng(pov.random_seed); // random generator

    std::vector<edgevis::Point> verticesPoly;
    std::vector<edgevis::Point> r_points;
    std::vector<polyanya::Point> p_points;

    edgevis::Point p;
    polyanya::Point anyaP;

    std::vector<Point> r_v;
    std::vector<Point> l_v;
    std::vector<Point> v;

    std::string name;
    parsers::GeomMesh geomMesh;
    if (pov.mesh_type == "polygonal") {
        geomMesh = geomMeshPoly;
    } else if (pov.mesh_type == "triangular") {
        geomMesh = geomMeshTri;
    } else {
        std::cout << "Unknown mesh type.\n";
        return -1;
    }
    geomMesh = IronHarvest;
    std::cout << "Tringular mesh has: " << geomMesh.polygons.size() << " triangles" << std::endl;
    std::cout << "Polygonal mesh has: " << geomMeshPoly.polygons.size() << " polygons" << std::endl;
    edgevis::Mesh edgemesh;
    edgemesh.read(geomMesh);
    edgemesh.precalc_point_location();
    edgemesh.calculate_edges();

    clock.Restart();
    for (int i = 0; i < pov.n_random_samples; i++) {
        auto rp = edgemesh.random_point(rng);
        p.x = rp.x;
        p.y = rp.y;
        anyaP.x = rp.x;
        anyaP.y = rp.y;
        r_points.push_back(p);
        p_points.push_back(anyaP);
    }
    time = clock.TimeInSeconds();
    std::cout << pov.n_random_samples << " random points generated in: " << time << " seconds." << std::endl;
    bool debug = pov.debug;
    bool save = pov.save;
    bool heatmap = pov.heatmap;
    double prepTime, prepOptim1Time;
    std::vector<double> times;
    double tMax = 0, tMin = 100, t;
    double edgePolyNodes, edgePolyNodesSum;
    double PolyNodesSum = 0;
    std::vector<polyanya::Point> verticesPolyAnya;
    double maxDepthSum = 0, expansionsSum = 0;
    std::ofstream results;
    results.open("results.dat", std::ios::out | std::ios::trunc);
    {
        PolyVis polyvisTriangular(geomMesh);
        edgevis::EdgeVisibility Evis(edgemesh);
        Evis.set_visual_mesh(geomMesh);

        std::cout << "\n\nMap: " << pov.input_map_name
                  << "\nType of mesh: " << pov.mesh_type
                  << "\nNumber of iterations: " << pov.n_random_samples;
        std::cout << "\n" << std::endl;

        clock.Restart();
        Evis.precompute_edges_searchnodes();

        time = clock.TimeInSeconds();
        prepTime = time;
        std::cout << "\nEdgevis\n";
        std::cout << "Preprocessing time for Edge Visibility was " <<
                  time << " seconds.\n";
        clock.Restart();
        Evis.precompute_edges_optimnodesV1();
        time = clock.TimeInSeconds();
        prepOptim1Time = time;
        std::cout << "Preprocessing time for OptimNodesV1 was " <<
                  time << " seconds.\n";

        clock.Restart();
        Evis.precompute_edges_optimnodesV2();
        time = clock.TimeInSeconds();
        prepOptim1Time = time;
        std::cout << "Preprocessing time for OptimNodesV2 was " <<
                  time << " seconds.\n";



        std::ofstream outfile;
        std::ofstream outfile_polyvis;

        if (!pov.machine) {
            results << pov.input_map_name
                    << "\nType of mesh: " << pov.mesh_type
                    << "\nNumber of iterations: " << pov.n_random_samples
                    << "\n=======================================================\n";
        } else {
            results << pov.input_map_name << "\n"
                    << pov.mesh_type << "\n"
                    << pov.n_random_samples << "\n";
        }

        if (save) {
            outfile.open("logger_edgevis.dat", std::ios::out | std::ios::trunc);
            outfile_polyvis.open("logger_polyvis.dat", std::ios::out | std::ios::trunc);
        }

        clock.Restart();
        int input;
        Point pos = r_points[0];
        bool end = false;
        int cEdge;
        if(debug){
            std::cout << "Controls: \n"
                      << " Move point: arrows (up, down, left, right) \n"
                      << " Zoom in: + \n"
                      << " Zoom out: -\n"
                      << " Change edge: Spacebar"
                      << " Quit: q \n";
            while(!end){
                verticesPoly = Evis.find_point_visibility_optim1(pos, debug, edgePolyNodes, cEdge);
                PolyNodesSum = PolyNodesSum + verticesPoly.size();
                edgePolyNodesSum = edgePolyNodesSum + edgePolyNodes;
                if(debug) {
                    Evis.visualise_point(pos, 0, false);
                    if(verticesPoly.size() > 0)
                        Evis.visualise_polygon(verticesPoly, 1, false);
                    Evis.visualiseOnline(pos);
                    input = cv::waitKey(30);
                    switch(input){
                        case 82:
                            pos.y = pos.y + 0.2;
                            break;
                        case 84:
                            pos.y = pos.y - 0.2;
                            break;
                        case 81:
                            pos.x = pos.x - 0.2;
                            break;
                        case 83:
                            pos.x = pos.x + 0.2;
                            break;
                        case 113:
                            end = true;
                            break;
                        case 43:
                            Evis.widthCV = Evis.widthCV * 0.9;
                            break;
                        case 45:
                            Evis.widthCV = Evis.widthCV * 1.1;
                            break;
                        case 32:
                            cEdge++;
                            break;
                        default:
                            break;
                    }
                }
            }
            return 1;
        }
        for (auto pos: r_points) {
            /*
            if(debug) {
                Evis.reset_visu();
                std::string m = pov.input_map_name + ".png";
                rename("debug_visu.png",  m.c_str());
            }
            */


            verticesPoly = Evis.find_point_visibility_optim1(pos, debug, edgePolyNodes, cEdge);
            PolyNodesSum = PolyNodesSum + verticesPoly.size();
            edgePolyNodesSum = edgePolyNodesSum + edgePolyNodes;
            /*
             * if(heatmap){
             *     t = clock.TimeInSeconds();
             *     times.push_back(t);
             *     tMax = std::max(tMax, t);
             *     tMin = std::min(tMin, t);
             * }
             *
             * if(save){
             *     outfile << "--\n";
             *     outfile << pos << std::endl;
             *     outfile << verticesPoly.size() << std::endl;
             *     for (auto p : verticesPoly){
             *         outfile << p << "; ";
             *     }
             *     outfile << std::endl;
             * }
             */
        }
        time = clock.TimeInSeconds();
        std::cout << "EdgeVis v1: " << "\n";
        std::cout << "Total computation time: " << time << " seconds.\n";
        std::cout << "Mean computation time: " << time / pov.n_random_samples << " seconds/point.\n";
        std::cout << "Mean number of nodes that were checked: " << edgePolyNodesSum / pov.n_random_samples << std::endl;
        std::cout << "Mean number of resulting visibility polygon nodes: " << PolyNodesSum / pov.n_random_samples
                  << std::endl;
        if (!pov.machine) {
            results << "EdgeVis:";
            results << "\nTotal computation time:" << time <<
                    "\nAverage computation time per point:" << time / pov.n_random_samples << std::endl;
        } else {
            results << time << " " <<
                    prepTime << " " <<
                    prepOptim1Time << " " <<
                    edgePolyNodesSum / pov.n_random_samples << " " <<
                    PolyNodesSum / pov.n_random_samples << "\n";
        }
        /*
         * if(heatmap)
         *     Evis.visualise_heatmap(r_points, times, tMax, tMin, "evis_heatmap.pdf");
         */
        PolyNodesSum = 0;
        edgePolyNodesSum = 0;
        times.clear();
        clock.Restart();
        for (auto pos: r_points) {
            verticesPoly = Evis.find_point_visibility_optim2(pos, debug, edgePolyNodes);
            PolyNodesSum = PolyNodesSum + verticesPoly.size();
            edgePolyNodesSum = edgePolyNodesSum + edgePolyNodes;
        }
        time = clock.TimeInSeconds();
        std::cout << "EdgeVis v2: " << "\n";
        std::cout << "Total computation time: " << time << " seconds.\n";
        std::cout << "Mean computation time: " << time / pov.n_random_samples << " seconds/point.\n";
        std::cout << "Mean number of nodes that were checked: " << edgePolyNodesSum / pov.n_random_samples << std::endl;
        std::cout << "Mean number of resulting visibility polygon nodes: " << PolyNodesSum / pov.n_random_samples
                  << std::endl;
        if (!pov.machine) {
            results << "EdgeVis:";
            results << "\nTotal computation time:" << time <<
                    "\nAverage computation time per point:" << time / pov.n_random_samples << std::endl;
        } else {
            results << time << " " <<
                    prepTime << " " <<
                    prepOptim1Time << " " <<
                    edgePolyNodesSum / pov.n_random_samples << " " <<
                    PolyNodesSum / pov.n_random_samples << "\n";
        }

        times.clear();
        clock.Restart();
        for (auto pos: p_points) {
            verticesPolyAnya = polyvisTriangular.get_visibility_polygon(pos);
            expansionsSum = expansionsSum + polyvisTriangular.expansions;
            maxDepthSum = maxDepthSum + polyvisTriangular.max_depth;
            /*
             * if(heatmap){
             *     t = clock.TimeInSeconds();
             *     times.push_back(t);
             *     tMax = std::max(tMax, t);
             *     tMin = std::min(tMin, t);
             * }
             * if(save){
             *     outfile_polyvis << "--\n";
             *     outfile_polyvis << pos << std::endl;
             *     outfile_polyvis << verticesPolyAnya.size() << std::endl;
             *     for (auto p : verticesPolyAnya){
             *         outfile_polyvis << p << "; ";
             *     }
             *     outfile_polyvis << std::endl;
             *
             * }
             */
        }
        time = clock.TimeInSeconds();
        std::cout << "\nPolyVis on triangles\n";
        std::cout << "Total computation time: " << time << " seconds.\n";
        std::cout << "Mean computation time: " << time / pov.n_random_samples << " seconds/point.\n";
        std::cout << "Mean number of edge expansions: " << expansionsSum / pov.n_random_samples << "\n";
        std::cout << "Mean maximal depth of expansion: " << maxDepthSum / pov.n_random_samples << "\n";

        if (!pov.machine) {
            results << "\n===============================================================\n";
            results << "PolyVis triangular:";
            results << "\nTotal computation time:" << time <<
                    "\nAverage computation time per point:" << time / pov.n_random_samples << std::endl;
        } else {
            results << time << " " << expansionsSum / pov.n_random_samples << " " << maxDepthSum / pov.n_random_samples
                    << "\n";
        }
    }

    PolyVis polyvisPolygonal(geomMeshPoly);
    // if(heatmap)
    //     Evis.visualise_heatmap(r_points, times, tMax, tMin, "poly_heatmap.pdf");

    edgevis::Mesh edgemesh2;
    edgemesh2.read(geomMeshPoly);
    edgemesh2.precalc_point_location();
    edgemesh2.calculate_edges();
    edgevis::EdgeVisibility Evis2(edgemesh2);
    Evis2.set_visual_mesh(geomMeshPoly);

    p_points.clear();
    for (int i = 0; i < pov.n_random_samples; i++) {
        auto rp = edgemesh2.random_point(rng);
        anyaP.x = rp.x;
        anyaP.y = rp.y;
        p_points.push_back(anyaP);
    }

    times.clear();
    clock.Restart();
    maxDepthSum = 0;
    expansionsSum = 0;
    int c;
    for (auto pos : p_points){
        if(c++ > 10000){
            c = 0;
            polyvisPolygonal.si->reset_node_pool();
        }
        pos.x = pos.x;
        pos.y = pos.y;
        verticesPolyAnya = polyvisPolygonal.get_visibility_polygon(pos);
        expansionsSum = expansionsSum + polyvisPolygonal.expansions;
        maxDepthSum = maxDepthSum + polyvisPolygonal.max_depth;

        if(false) {
            Evis2.reset_visu();
            Evis2.visualise_point({pos.x, pos.y}, 0, true);

            verticesPoly.resize(verticesPolyAnya.size());
            for (int i = 0; i < verticesPolyAnya.size(); i++) {
                verticesPoly[i].x = verticesPolyAnya[i].x;
                verticesPoly[i].y = verticesPolyAnya[i].y;
            }
            Evis2.visualise_polygon(verticesPoly, 1, true);
            getchar();
        }
        //if(heatmap){
        //    t = clock.TimeInSeconds();
        //    times.push_back(t);
        //    tMax = std::max(tMax, t);
        //    tMin = std::min(tMin, t);
        //}
        //if(save){
        //    outfile_polyvis << "--\n";
        //    outfile_polyvis << pos << std::endl;
        //    outfile_polyvis << verticesPolyAnya.size() << std::endl;
        //    for (auto p : verticesPolyAnya){
        //        outfile_polyvis << p << "; ";
        //    }
        //    outfile_polyvis << std::endl;
        //
        //}
    }
    time = clock.TimeInSeconds();
    std::cout << "\nPolyVis on polygons\n";
    std::cout << "Total computation time: "<< time << " seconds.\n";
    std::cout << "Mean computation time: "<< time/pov.n_random_samples << " seconds/point.\n";
    std::cout << "Mean number of edge expansions: " << expansionsSum/pov.n_random_samples << "\n";
    std::cout << "Mean maximal depth of expansion: " << maxDepthSum/pov.n_random_samples << "\n";

    if(!pov.machine){
        results << "\n===============================================================\n";
        results << "PolyVis polygonal:";
        results << "\nTotal computation time:" << time <<
                "\nAverage computation time per point:" << time/pov.n_random_samples << std::endl;
    }else{
        results << time << " " << expansionsSum/pov.n_random_samples << " " << maxDepthSum/pov.n_random_samples << "\n";
    }


    //if(heatmap)
    //    Evis.visualise_heatmap(r_points, times, tMax, tMin, "poly_heatmap.pdf");

    results.close();
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
