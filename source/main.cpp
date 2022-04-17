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

    polyanya::Point p;
    p.x = 10;
    p.y = 10;
    PolyVis solver(geomMesh);
    solver.switch_measurement(true, true);
    std::vector<polyanya::Point> vertices = solver.get_visibility_polygon(p);

    std::vector<int> m = solver.read_measurements();
    std::cout << m[0] << std::endl;

    MapVisualizer drawer(geomMesh);
    //drawer.parse_mesh();
    drawer.set_visible_polygon(p, vertices);
    drawer.redraw();

	return 0;
}