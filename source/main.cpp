#include "visualization.h"
#include "polyviz.h"
#include "polyanya/parsers/libs/map_parser.h"

#include <chrono>


int main(int argc, const char *const *argv)
{
    std::string potholeName = "../maps/potholes/potholes.txt";
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
    mapParser.convertFade2DMeshToGeomMesh(fade2DMesh, geomMesh);

    std::vector<polyanya::Point> vertices;

    PolyVis solver(geomMesh);
    solver.switch_measurement(true, true);
    std::vector<polyanya::Point> positions = solver.generate_points(1000);
    for (auto p : positions){
        vertices = solver.get_visibility_polygon(p);
    }

    std::vector<int> m = solver.read_measurements();

    for (auto s : m){
        std::cout << s << ", ";
    }
    std::cout << std::endl;

    MapVisualizer drawer(geomMesh);
    drawer.set_visible_polygon(positions.back(), vertices);
    drawer.redraw();

	return 0;
}