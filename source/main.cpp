#include "visualization.h"
#include "polyviz.h"
#include "polyanya/parsers/map_parser.h"

#include <chrono>
#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif

int main(int argc, const char *const *argv)
{
    std::string map_name = "/potholes.txt";
    std::string map_directory = INPUT_MAPS_DIR;
    std::string potholeName = map_directory + map_name;
    // Define map parser class
    parsers::MapParser mapParser;
    parsers::Fade2DMesh fade2DMesh;
    parsers::MergedMesh mergedMesh;
    parsers::GeomMesh geomMesh;
    // Define paths
    std::string fade2dMeshPath = "";
    std::string mergedMeshPath = "";

    // Convert map to fade2d mesh
    mapParser.convertMapToFade2DMesh(potholeName, fade2DMesh);
    mapParser.convertMapToMergedMesh(potholeName, mergedMesh);
    mapParser.convertMergedMeshToGeomMesh(mergedMesh, geomMesh);
    //mapParser.convertFade2DMeshToGeomMesh(fade2DMesh, geomMesh);

    std::vector<polyanya::Point> vertices;

    PolyVis solver(geomMesh);
    solver.switch_measurement(true, true);
    std::vector<polyanya::Point> positions = solver.generate_points(1000);
    for (auto p : positions){
        vertices = solver.get_visibility_polygon(p);
    }

    std::vector<int> m = solver.read_measurements();
    float sum = 0;
    for (auto s : m){
        sum = sum + s;
    }
    std::cout << sum / 1000 << std::endl;

    polyanya::Point p;

    p.x = 18.87969970703125;
    p.y = 28.160797119140625;
    vertices = solver.get_visibility_polygon(p);
    MapVisualizer drawer(geomMesh);
    drawer.set_visible_polygon(p, vertices);
    drawer.redraw();

	return 0;
}