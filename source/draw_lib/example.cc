
#include <iostream>

#include "geom/geom.h"
#include "geom/utils.h"
#include "cairo_geom_drawer/cairo_geom_drawer.h"
#include "polygon_clipping/polygon_clipping.h"
#include "polygonal_map/polygonal_map.h"

namespace clip = polygon_clipping;
namespace cgm = cairo_geom_drawer;
namespace map = polygonal_map;
namespace draw = map::draw;

/// Example of using polygons drawing tool
int main(int argc, const char *const *argv) {

    /// Basics of geometry.
    geom::Point<double> p1 = {0, 1};
    geom::Point<double> p2 = {1, 0};
//    double dist = p1.DistanceTo(p2);
    auto p3 = p1 + p2;
    std::cout << p3 << std::endl;
    std::cout << "(" << p3.ToString(", ") << ")" << std::endl;
    double x = 0.0;
    double y = 3.0;
    geom::Point<double> p4(x, y);
    auto p5 = geom::MakePoint(x, y);

    /// Basic drawing.
    geom::Polygon<double> border = {{2,  1},
                                    {20, 0},
                                    {21, 20},
                                    {0,  19}};
    geom::Polygon<double> obstacle1 = {{6, 12},
                                       {7, 12},
                                       {7, 7},
                                       {5, 7}};
    geom::Polygon<double> obstacle2 = {{11, 7},
                                       {13, 8},
                                       {11, 2}};
    geom::Polygon<double> obstacle3 = {{16, 18},
                                       {18, 11},
                                       {15, 13},
                                       {10, 10}};
    geom::Polygons<double> obstacles = {obstacle1, obstacle2, obstacle3};
    geom::Polygons<double> simple_map = {border};
    simple_map.insert(simple_map.end(), obstacles.begin(), obstacles.end());

    /// Create draw_lib
    double x_min, x_max, y_min, y_max;
    geom::ComputeLimits(simple_map, x_min, x_max, y_min, y_max);
    cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);

    /// Create and save to PDF file
    cgm_drawer.OpenPDF("simple_map.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorWhite);
    cgm_drawer.DrawPolygons(obstacles, cgm::kColorBlack);
    cgm_drawer.Close();

    /// Polygon clipping.
    geom::Polygon<double> poly1 = {{16, 4},
                                   {18, 7},
                                   {18, 9},
                                   {15, 7}};
    geom::Polygon<double> poly2 = {{17, 4},
                                   {17, 8},
                                   {15, 6},
                                   {15, 3}};
    cgm_drawer.OpenPDF("poly_clipping.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorWhite);
    cgm_drawer.DrawPolygons(obstacles, cgm::kColorBlack);
    clip::CPolygons solution;
    clip::Clip(clip::CClipType::ctIntersection, clip::Geom2Clipper(poly1), clip::Geom2Clipper(poly2), solution);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(solution), cgm::kColorOrange);
    clip::Clip(clip::CClipType::ctDifference, clip::Geom2Clipper(poly1), clip::Geom2Clipper(poly2), solution);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(solution), cgm::kColorRed, 0.5);
    cgm_drawer.DrawPolygon(poly1, 0.1, cgm::kColorLimeGreen);
    cgm_drawer.DrawPolygon(poly2, 0.1, cgm::kColorDeepSkyBlue);
    cgm_drawer.DrawPoint({17, 4}, 0.5, cgm::kColorRed, 0.5);
    std::cout << "Orientations: ";
    for (const auto &p : simple_map) std::cout << clip::Orientation(clip::Geom2Clipper(p)) << " ";
    std::cout << std::endl;
    std::cout << "Relative area: " << clip::Area(clip::Geom2Clipper(simple_map)) << std::endl;
    auto simple_map_clip_offset = clip::Geom2Clipper(simple_map);
    clip::Offset(simple_map_clip_offset, -0.5);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(simple_map_clip_offset), 0.1, cgm::kColorSilver);
    cgm_drawer.Close();

    /// Load the map safely.
    map::PolygonalMap poly_map;
    std::string file_name = std::string(MAP_DATA_DIR) + "/jari-huge/jari-huge.txt";
    //if (poly_map.LoadSafely(file_name) == map::PolygonalMap::LoadCode::kOK) {
    //    // do sth
    //} else {
    //    // error
    //}
    auto load_code = poly_map.LoadSafely(file_name);
    switch (load_code) {
        case map::PolygonalMap::LoadCode::kOK: {
            std::cout << "OK: File " << file_name << " loaded." << std::endl;
            break;
        }
        case map::PolygonalMap::LoadCode::kNotOpenedOrFound: {
            std::cout << "ERROR: File " << file_name << " could not opened or found!" << std::endl;
            return 1;
        }
        case map::PolygonalMap::LoadCode::kInvalid: {
            std::cout << "ERROR: File " << file_name << " is invalid!" << std::endl;
            return 1;
        }
    }

    /// Standardize the map (important for good functionality).
    poly_map.Standardize(0.02, 1e6);

    /// Check if valid.
    std::cout << "Validity: " << static_cast<int>(poly_map.Valid()) << std::endl;

    /// Draw the map.
    // poly_map.set_polygons(simple_map);
    auto drawer = poly_map.CreateMapDrawer();
    drawer.OpenPDF("map.pdf");
    drawer.DrawPlane(draw::kColorDeepSkyBlue);
    drawer.DrawBorders(draw::kColorOrange);
    drawer.DrawObstacles(draw::kColorLimeGreen);
    // drawer.DrawMap();
    drawer.Close();

    // Example number two
    //    std::vector<double> thetas;
//    std::vector<geom::Point<double>> points;
//    std::vector<geom::Point<double>> pointsFromTh;
//    readResults(thetas, points);
//
//    geom::Point<double> a = {-1 + PX, 3 + PY};
//    geom::Point<double> b = {1 + PX, 3 + PY};
//
//    double radius = 1.0;
//    geom::Point<double> p = {PX, PY};
//    geom::Circle<double> circle(radius, p);
//    geom::Point<double> optimalPoint;
//    geom::Point<double> reflectivePoint;
//
//    computeOptimalPoint(points, a, b, optimalPoint);
//    computeReflectivePoints(optimalPoint, a, reflectivePoint);
//    computePointsFromThetas(pointsFromTh, thetas, circle);
//
//    double distA = computeDistance(optimalPoint, a) + computeDistance(optimalPoint, b);
//    double distAStar = computeDistance(optimalPoint, reflectivePoint) + computeDistance(optimalPoint, b);
//
//    std::cout << "Point A: " << a << std::endl;
//    std::cout << "Point A*: " << reflectivePoint << std::endl;
//    std::cout << "Point P: " << optimalPoint << std::endl;
//    std::cout << "Distance A: " << distA << std::endl;
//    std::cout << "Distance A star: " << distAStar << std::endl;
//
//    for (int i = 0; i < pointsFromTh.size(); ++i) {
//        std::cout << i + 1 << ", Point:" << points[i] << ", PointFromTh: " << pointsFromTh[i] << std::endl;
//    }
//
//
////    CCircle circleClass(a, b);
//
////    geom::Points<double> points = circleClass.computePoints(circle);
////    std::vector<double> angles = circleClass.computeAngles(circle);
//
////    for (auto &angle : angles) {
////        std::cout << angle << std::endl;
//////        cgm_drawer.DrawPoint(point, 0.2, cgm::kColorBlack, 1.0);
////    }
//
//
//    geom::Polygon<double> border = {{0,  0},
//                                    {20, 0},
//                                    {20, 20},
//                                    {0,  20}};
//    geom::Polygons<double> simple_map = {border};
//
//    /// Create draw_lib
//    double x_min, x_max, y_min, y_max;
//    geom::ComputeLimits(simple_map, x_min, x_max, y_min, y_max);
////    x_max = 10;
////    y_max = 10;
//    cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);
//
//    /// Create and save to PDF file
//    cgm_drawer.OpenPDF("simple_map_circle.pdf");
//    cgm_drawer.DrawPlane(cgm::kColorBlack);
//    cgm_drawer.DrawPolygon(border, cgm::kColorWhite);
//    cgm_drawer.DrawPoint(circle.center, circle.radius, cgm::kColorLimeGreen, 0.2);
//    cgm_drawer.DrawPoint(a, 0.1, cgm::kColorGreen, 0.8);
//    cgm_drawer.DrawPoint(b, 0.1, cgm::kColorRed, 0.8);
//    cgm_drawer.DrawPoint(reflectivePoint, 0.1, cgm::kColorBlue, 0.8);
//
////    geom::Point<double> man = {0 + PX, 1 + PY};
////    cgm_drawer.DrawPoint(man, 0.1, cgm::kColorPurple, 1.0);
//
//    cgm_drawer.DrawPoint(optimalPoint, 0.1, cgm::kColorBlack, 0.8);
//    for (auto &point : points) {
//        std::cout << point << std::endl;
//        cgm_drawer.DrawPoint(point, 0.1, cgm::kColorLime, 0.2);
//    }
//    cgm_drawer.Close();
    return 0;
}