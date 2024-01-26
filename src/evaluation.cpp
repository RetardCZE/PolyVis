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
    const std::string mapsDir = "../data/maps/";
    const std::string pointDir = "../data/points/";
    const std::string resultsDir = "../data/results/";
    std::vector<std::string> mapNames;
    std::vector<std::string> pointTypes = {// "_points_close_to_edge_mid_points",
                                           // "_points_close_to_nodes",
                                           // "_points_edge_mid_points",
                                           // "_points_on_nodes",
                                           //"_points_random",
                                           "_points_random_interior"};
    std::vector<std::string> algorithms = {"_TEA",
                                           "_PEA",
                                           "_Edgevis"};

    int counter = 0;
    try {
        for (auto& entry : fs::directory_iterator(mapsDir)) {
            if (is_regular_file(entry)) {
                mapNames.push_back(entry.path().stem().string());
            }
        }
        for(auto mapName: mapNames){
            counter++;
            std::cout << mapName <<std::endl;
            for(auto pointFile : pointTypes){

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
                std::vector<std::ifstream> results(algorithms.size());
                std::vector<double> time_init;
                std::vector<double> time_preprocess;
                std::istringstream iss;
                for( int a = 0; a < algorithms.size(); a++){
                    results[a].open(resultsDir + mapName + pointFile + algorithms[a] + ".txt");
                    std::getline(results[a], line); // time init
                    iss.str(line);
                    iss >> word;
                    iss >> word;
                    time_init.push_back(std::stod(word));
                    iss.clear();
                    std::getline(results[a], line); // time preprocess
                    iss.str(line);
                    iss >> word;
                    iss >> word;
                    time_preprocess.push_back(std::stod(word));
                    iss.clear();
                    std::getline(results[a], line); // point count
                }
                //std::cout << mapName << " " << time_preprocess[0] << " " << time_preprocess[1] << " " << time_preprocess[2] << std::endl;
                std::vector<std::vector<edgevis::Point>> polygons(algorithms.size()); //add more for more options
                // std::vector<double> time;
                ClipperLib::Clipper clipper;
                std::vector<ClipperLib::Path> paths(algorithms.size());
                ClipperLib::IntPoint cp;
                int cntr;
                for(int i = 0; i<pointCount; i++){
                    for( int a = 0; a < algorithms.size(); a++){
                        polygons[a].clear();
                        paths[a].clear();
                        std::getline(results[a], line);
                        iss.str(line);
                        iss >> word; // point id
                        iss >> word; // times
                        iss >> word; // num of times
                        cntr = std::stoi(word);
                        for(int j = 0; j<cntr; j++) {
                            iss >> word; // times 1 to n
                        }
                        iss >> word; // polygon
                        iss >> word; // num of points in polygon
                        cntr = std::stoi(word);
                        for(int j = 0; j<cntr; j++) {
                            iss >> word; // x
                            p.x = std::stod(word);
                            iss >> word; // y
                            p.y = std::stod(word);
                            polygons[a].push_back(p);
                            cp.X = p.x;
                            cp.Y = p.y;
                            paths[a].push_back(cp);
                        }
                        iss.clear();
                    }
                    ClipperLib::Paths xorPath;

                    std::vector<double> areas(algorithms.size()); // first is area of reference then relative error of other polygons
                    std::vector<double> XOR_areas(algorithms.size());
                    for( int a = 0; a < algorithms.size(); a++) {
                        areas[a] = ClipperLib::Area(paths[a]);
                    }

                    clipper.Clear();
                    xorPath.clear();
                    clipper.AddPath(paths[0], ClipperLib::ptSubject, true);
                    clipper.AddPath(paths[1], ClipperLib::ptClip, true);
                    clipper.Execute(ClipperLib::ctXor, xorPath);
                    for (const auto& dPath : xorPath) {
                        XOR_areas[0] += ClipperLib::Area(dPath);
                    }

                    clipper.Clear();
                    xorPath.clear();
                    clipper.AddPath(paths[1], ClipperLib::ptSubject, true);
                    clipper.AddPath(paths[2], ClipperLib::ptClip, true);
                    clipper.Execute(ClipperLib::ctXor, xorPath);
                    for (const auto& dPath : xorPath) {
                        XOR_areas[1] += ClipperLib::Area(dPath);
                    }

                    clipper.Clear();
                    xorPath.clear();
                    clipper.AddPath(paths[2], ClipperLib::ptSubject, true);
                    clipper.AddPath(paths[0], ClipperLib::ptClip, true);
                    clipper.Execute(ClipperLib::ctXor, xorPath);
                    for (const auto& dPath : xorPath) {
                        XOR_areas[2] += ClipperLib::Area(dPath);
                    }
                    std::cout << "___________________________________________________________________" << std::endl;
                    std::cout << "polygon: " << i << std::endl;
                    std::cout << areas[0] << " | " << areas[1] << " | " << areas[2] << std::endl;
                    std::cout << XOR_areas[0] << " | " << XOR_areas[1] << " | " << XOR_areas[2] << std::endl;
                    std::cout << "___________________________________________________________________" << std::endl;

                    parsers::GeomMesh gMesh = load_mesh(mapsDir + mapName + ".txt", false);
                    edgevis::Mesh mesh(gMesh);
                    mesh.reset_visu();
                    if(polygons[0].size()>0){
                        mesh.visualise_polygon(polygons[0], 0, false);
                        mesh.visualise_polygon(polygons[1], 1, false);
                        mesh.visualise_polygon(polygons[2], 2, false);
                        mesh.visualise_point(points[i], 0, true);
                    }
                    getchar();
                }
                for( int a = 0; a < algorithms.size(); a++){
                    results[a].close();
                }
                break;
            }
            break;
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
