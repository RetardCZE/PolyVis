/**
 * File:    parser.cpp
 *
 * Date:   04.04.2021
 * Author:  Lukas Fanta
 * E-mail:  fantalukas2108@gmail.com
 *
 */

#include "libs/map_parser.h"
#include "mesh_plotter/plotter.h"

int main(int argc, const char *const *argv) {

    std::string potholeName = "../../../../data/maps/potholes/potholes.txt";
    // Define map parser class
    parsers::MapParser mapParser;
    // Define structs
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh geomMesh;

    // Define paths
    std::string fade2dMeshPath = "";
    std::string mergedMeshPath = "";

    // Convert map to fade2d mesh
    mapParser.convertMapToFade2DMesh(potholeName, fade2DMesh);

    // Print and draw fade 2d mesh structure
    mesh_plotter::printFade2DMesh(fade2DMesh, std::cout);
    mesh_plotter::drawFade2D(fade2DMesh, fade2dMeshPath);

    // Direct conversion of map to merged mesh
    mapParser.convertMapToMergedMesh(potholeName, mergedMesh);

    // Print and draw merged mesh
    mesh_plotter::printMergedMesh(mergedMesh, std::cout);
    mesh_plotter::drawMergedMesh(mergedMesh, mergedMeshPath);

    // Create geom mesh (used in distance solver class) from merged mesh
    mapParser.convertMergedMeshToGeomMesh(mergedMesh, geomMesh);

    // Print geom mesh
    mesh_plotter::printGeomMesh(geomMesh, std::cout);

    return 0;
}