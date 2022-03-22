#include "draw_lib/geom/geom.h"
#include "draw_lib/geom/utils.h"
#include "draw_lib/cairo_geom_drawer/cairo_geom_drawer.h"
#include "draw_lib/polygon_clipping/polygon_clipping.h"
#include "draw_lib/polygonal_map/polygonal_map.h"

//include "polyanya"
#include "polyanya/helpers/scenario.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

namespace clip = polygon_clipping;
namespace cgm = cairo_geom_drawer;
namespace map = polygonal_map;
namespace draw = map::draw;

int main(int argc, const char *const *argv);
