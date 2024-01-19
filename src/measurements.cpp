//
// Created by jakub on 19.1.24.
//
// === THIS PROJECT INCLUDES ===
#include "edgevis/structs/mesh.h"
#include "polyanya/search/polyviz.h"

#include "geomMesh/parsers/map_parser.h"

// #include "utils/simple_clock.h"

#include <iomanip>
#include <filesystem>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "edgevis/libs/clipper/clipper.hpp"

#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif
#ifndef INPUT_CONVERTED_MAPS_DIR
#define INPUT_CONVERTED_MAPS_DIR "."
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace cgm = cairo_geom_drawer;

parsers::GeomMesh
load_mesh(std::string map_full_path, bool polygonal){
    parsers::MapParser mapParser;
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh result;
    int obstacles;
    double time;
    custom::utils::SimpleClock clock;
    clock.Restart();
    if(!polygonal){
        mapParser.convertMapToFade2DMesh(map_full_path, fade2DMesh, obstacles);
        mapParser.convertFade2DMeshToGeomMesh(fade2DMesh, result);
        time = clock.TimeInSeconds();
        //std::cout << "[CDT]-" << time << std::endl;
    }else{
        mapParser.convertMapToMergedMesh(map_full_path, mergedMesh);
        mapParser.convertMergedMeshToGeomMesh(mergedMesh, result);
        time = clock.TimeInSeconds();
        //std::cout << "[M-CDT]-" << time << std::endl;
    }
    return result;
}

int body(){
    const std::string mapsDir = "/home/jakub/Projects/gitEdgeVis/testing/data/maps/";
    const std::string pointDir = "/home/jakub/Projects/gitEdgeVis/testing/data/points/";
    const std::string resultsDir = "/home/jakub/Projects/gitEdgeVis/testing/data/results/";
    std::vector<std::string> mapNames;
    std::vector<std::string> pointTypes = {// "_points_close_to_edge_mid_points",
                                           // "_points_close_to_nodes",
                                           // "_points_edge_mid_points",
                                           // "_points_on_nodes",
                                           "_points_random",
                                           "_points_random_interior"};

    int counter = 0;
    try {
        for (auto& entry : fs::directory_iterator(mapsDir)) {
            if (is_regular_file(entry)) {
                mapNames.push_back(entry.path().stem().string());
            }
        }
        for(auto mapName: mapNames){
            counter++;
            for(auto pointFile : pointTypes){
                //mapName = "scene_sp_rus_02";
                std::string pointPath = pointDir + mapName + pointFile + ".txt";
                std::ifstream inputFile(pointPath);

                std::string line;
                std::string word;
                std::getline(inputFile, line);
                std::vector<edgevis::Point> points;
                edgevis::Point p;
                int pointCount = std::stoi(line);
                for(int i = 0; i<pointCount; i++){
                    std::getline(inputFile, line);
                    std::istringstream iss(line);
                    iss >> word;
                    iss >> word;
                    p.x = std::stod(word);
                    iss >> word;
                    p.y = std::stod(word);
                    points.push_back(p);
                }
                inputFile.close();
                {
                    std::cout << mapName << " | " << pointFile << " | TEA" <<std::endl;
                    std::string output = resultsDir + mapName + pointFile + "_Rosol_TEA_visibility.txt";
                    std::ofstream outputFile(output);
                    outputFile << std::fixed << std::setprecision(17);

                    custom::utils::SimpleClock clock;
                    double time_init, time_preprocess, T1, T2, T3;
                    parsers::GeomMesh gMesh = load_mesh(mapsDir + mapName + ".txt", false);
                    clock.Restart();
                    edgevis::Mesh mesh(gMesh);
                    mesh.TEAItems = 200;
                    mesh.realloc_TEA_mem(mesh.TEAItems);
                    mesh.useRobustOrientatation = true;
                    time_init = clock.TimeInSeconds();
                    time_preprocess = 0.0;
                    outputFile << "time_init " << time_init << '\n';
                    outputFile << "time_preprocess " << time_preprocess << '\n';
                    outputFile << points.size() << std::endl;
                    clock.Restart();
                    std::vector<edgevis::Point> visibility;

                    for (int j = 0; j < points.size(); j++) {
                        visibility = mesh.find_point_visibility_TEA(points[j], T1, T2, T3);
                        outputFile << j << " times" << " 3" << " " << T1 << " " << T2 << " " << T3;
                        outputFile << " polygon " << visibility.size();
                        for (auto &p: visibility) {
                            outputFile << " " << p.x << " " << p.y;
                        }
                        outputFile << "\n";
                    }
                    mesh.allocTEA.clear();
                    outputFile.close();
                }

                {
                    std::cout << mapName << " | " << pointFile << " | PEA" <<std::endl;
                    std::string output = resultsDir + mapName + pointFile + "_Rosol_PEA_visibility.txt";
                    std::ofstream outputFile(output);
                    outputFile << std::fixed << std::setprecision(17);

                    custom::utils::SimpleClock clock;
                    double time_init, time_preprocess, T1, T2, T3;
                    parsers::GeomMesh gMesh = load_mesh(mapsDir + mapName + ".txt", true);
                    clock.Restart();
                    edgevis::Mesh mesh(gMesh);
                    mesh.PEAItems = 200;
                    mesh.realloc_PEA_mem(mesh.PEAItems);
                    mesh.useRobustOrientatation = true;
                    time_init = clock.TimeInSeconds();
                    time_preprocess = 0.0;
                    outputFile << "time_init " << time_init << '\n';
                    outputFile << "time_preprocess " << time_preprocess << '\n';
                    outputFile << points.size() << std::endl;
                    clock.Restart();
                    std::vector<edgevis::Point> visibility;

                    for (int j = 0; j < points.size(); j++) {
                        //std::cout << j << std::endl;
                        visibility = mesh.find_point_visibility_PEA(points[j], T1, T2, T3);
                        outputFile << j << " times" << " 3" << " " << T1 << " " << T2 << " " << T3;
                        outputFile << " polygon " << visibility.size();
                        for (auto &p: visibility) {
                            outputFile << " " << p.x << " " << p.y;
                        }
                        outputFile << "\n";
                    }
                    mesh.allocPEA.clear();
                    outputFile.close();
                }

                {
                    std::cout << mapName << " | " << pointFile << " | EdgeVis" <<std::endl;
                    std::string output = resultsDir + mapName + pointFile + "_Rosol_EdgeVis_visibility.txt";
                    std::ofstream outputFile(output);
                    outputFile << std::fixed << std::setprecision(17);

                    custom::utils::SimpleClock clock;
                    double time_init, time_preprocess, T1, T2, T3;
                    parsers::GeomMesh gMesh = load_mesh(mapsDir + mapName + ".txt", false);
                    clock.Restart();
                    edgevis::Mesh mesh(gMesh);
                    mesh.useRobustOrientatation = true;
                    time_init = clock.TimeInSeconds();
                    clock.Restart();
                    mesh.precompute_edges_searchnodes();
                    mesh.precompute_edges_optimnodesV1();
                    time_preprocess = clock.TimeInSeconds();
                    outputFile << "time_init " << time_init << '\n';
                    outputFile << "time_preprocess " << time_preprocess << '\n';
                    outputFile << points.size() << std::endl;
                    clock.Restart();
                    std::vector<edgevis::Point> visibility;

                    for (int j = 0; j < points.size(); j++) {
                        visibility = mesh.find_point_visibility_optim1(points[j], T1, T2, T3);
                        outputFile << j << " times" << " 2" << " " << T1 << " " << T2;
                        outputFile << " polygon " << visibility.size();
                        for (auto &p: visibility) {
                            outputFile << " " << p.x << " " << p.y;
                        }
                        outputFile << "\n";
                    }
                    outputFile.close();
                }
            }
            if(counter >= 5) break;
        }
    } catch (const fs::filesystem_error& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}

int main(
        int argc,
        const char *const *argv
) {
    body();
}
