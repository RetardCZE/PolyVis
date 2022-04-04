#include "visualization.h"
#include "polyviz.h"

int main(int argc, const char *const *argv)
{
    std::string file = "../polyanya/meshes/arena-merged.mesh";
    std::ifstream meshfile(file);

    polyanya::Point p;
    p.x = 10;
    p.y = 10;
    PolyVis solver(file);
    std::vector<polyanya::Point> vertices = solver.get_visibility_polygon(p);

    MapVisualizer drawer(file);
    drawer.parse_mesh();
    drawer.set_visible_polygon(p, vertices);
    drawer.redraw();

	return 0;
}