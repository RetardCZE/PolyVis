/**
 * File:    polygonal_map.cc
 *
 * Date:    19.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#include "../polygonal_map/polygonal_map.h"

#include <fstream>
#include <limits>

#include "../polygon_clipping/polygon_clipping.h"
#include "../geom/utils.h"

#include <iostream>

using namespace geom;
using namespace polygonal_map;
namespace clip = polygon_clipping;

std::ostream &polygonal_map::operator<<(std::ostream &os, const Polygon<double> &polygon) {
    for (const auto &p : polygon) os << std::string(((p == polygon[0]) ? "" : "\n")) << p;
    return os;
}

std::ostream &polygonal_map::operator<<(std::ostream &os, const Polygons<double> &polygons) {
    for (const auto &poly : polygons) os << std::string(((poly == polygons[0]) ? "" : "\n\n")) << poly;
    return os;
}

PolygonalMap::ValidityCode PolygonalMap::Valid(const geom::Polygons<double> &polygons) {
    // Check if not empty.
    if (polygons.empty()) return ValidityCode::kEmpty;
    // Extract border and obstacles.
    Polygons<double> borders;
    Polygons<double> obstacles;
    auto clip_polygons = clip::Geom2Clipper(polygons);
    for (long unsigned int kI = 0; kI < polygons.size(); ++kI) {
        if (clip::Orientation(clip_polygons[kI])) borders.push_back(polygons[kI]);
        else
            obstacles.push_back(polygons[kI]);
    }
    // Check if at least one border
    if (borders.empty()) return ValidityCode::kNoBorder;
    // Check if only one border.
    if (borders.size() > 1) return ValidityCode::kMoreBorders;
    // Extract border as single polygon.
    auto border = Polygon<double>(borders[0].begin(), borders[0].end());
    // Create clip versions of border and obstacles.
    auto clip_border = clip::Geom2Clipper(border);
    auto clip_obstacles = clip::Geom2Clipper(obstacles);
    // Check if border is not empty.
    if (border.empty()) return ValidityCode::kEmptyBorder;
    // Check if all obstacles are not empty.
    for (const auto &o : obstacles)
        if (o.empty()) return ValidityCode::kEmptyObstacle;
    // Check if border is a polygon (at least 3 points).
    if (border.size() < 3) return ValidityCode::kNotPolygonBorder;
    // Check if all obstacles are polygons (at least 3 points).
    for (const auto &o : obstacles) {
        if (o.size() < 3) return ValidityCode::kNotPolygonObstacle;
    }
    // Check if all obstacles are inside the border (all their points must be in or on border).
    for (const auto &c_obstacle : clip_obstacles) {
        for (const auto &c_p : c_obstacle) {
            if (!(clip::PointInPolygon(c_p, clip_border) || clip::PointOnPolygon(c_p, clip_border)))
                return ValidityCode::kObstacleOutsideBorder;
        }
    }
    // Check if all pairs of obstacles have empty intersection.
    for (const auto &c_obstacle_1 : clip_obstacles) {
        for (const auto &c_obstacle_2 : clip_obstacles) {
            if (c_obstacle_1 != c_obstacle_2) {
                clip::CPolygons sol;
                clip::Clip(clip::CClipType::ctIntersection, c_obstacle_1, c_obstacle_2, sol);
                if (!sol.empty()) return ValidityCode::kNonEmptyObstaclePairIntersection;
            }
        }
    }
    // TODO: check if polygons are simple (see SimplifyPolygons in ClipperLib).
    return ValidityCode::kOK;
}

void PolygonalMap::Fix(Polygons<double> &polygons) {
    auto simplified_polygons = clip::Geom2Clipper(polygons);
    clip::SimplifyPolygons(simplified_polygons);
    clip::CleanPolygons(simplified_polygons);
    polygons = clip::Clipper2Geom<double>(simplified_polygons);
}

template<typename T>
void ChangeOrientation(geom::Polygon<T> &poly) {
    geom::Polygon<T> poly_out;
    for (auto i = static_cast<int>(poly.size()) - 1; i >= 0; --i) poly_out.emplace_back(poly[i]);
    poly = poly_out;
}

bool PolygonalMap::Load(const std::string &file, Polygons<double> &polygons) noexcept(false) {
    std::ifstream ifs(file.c_str());
    if (ifs.fail()) return false;
    std::string token;
    double scale = 1.0;
    geom::Polygon<double> curr_polygon;
    bool processing_border = false;
    while (true) {
        ifs >> token;
        // Save the border flag.
        if (token == "[BORDER]") processing_border = true;
        // If the next border/obstacle polygon or the end of the file is detected, ...
        // ... then save the current polygon to the map if it is non-empty.
        if (token == "[BORDER]" || token == "[OBSTACLE]" || ifs.eof()) {
            if (!curr_polygon.empty()) {
                // Check polygon orientations.
                if (processing_border) {
                    // Border should return true.
                    if (!clip::Orientation(clip::Geom2Clipper(curr_polygon))) {
                        // Fix the orientation if wrong.
                        ChangeOrientation(curr_polygon);
                    }
                    processing_border = false;
                } else {
                    // Obstacle should return false.
                    if (clip::Orientation(clip::Geom2Clipper(curr_polygon))) {
                        // Fix the orientation if wrong.
                        ChangeOrientation(curr_polygon);
                    }
                }
                // Add the polygon to the map.
                polygons.push_back(curr_polygon);
                curr_polygon.clear();
                if (ifs.eof()) break;
            }
        } else if (token == "[SCALE]") {
            ifs >> token;
            scale = std::stod(token);
        } else if (!ifs.eof()) { // Assuming line with two double coordinates.
            double x, y;
            x = std::stod(token) * scale;
            ifs >> token;
            y = std::stod(token) * scale;
            curr_polygon.emplace_back(x, y);
        }
    }
    return true;
}

PolygonalMap::LoadCode PolygonalMap::LoadSafely(const std::string &file, Polygons<double> &polygons) {
    try {
        if (!Load(file, polygons)) return LoadCode::kNotOpenedOrFound;
    } catch (std::invalid_argument &) {
        return LoadCode::kInvalid;
    }
    return LoadCode::kOK;
}

void PolygonalMap::Shift(const double &shift_x, const double &shift_y, Polygons<double> &polygons) {
    for (auto &poly : polygons) {
        for (auto &p : poly) {
            p.x += shift_x;
            p.y += shift_y;
        }
    }
}

void PolygonalMap::Round(const double &rounder, Polygons<double> &polygons) {
    for (auto &poly : polygons) {
        for (auto &p : poly) {
            p.x = std::round(p.x * rounder) / rounder;
            p.y = std::round(p.y * rounder) / rounder;
        }
    }
}

Limits PolygonalMap::ComputeLimits(const Polygons<double> &polygons) {
    Limits lim;
    geom::ComputeLimits(polygons, lim.x_min, lim.x_max, lim.y_min, lim.y_max);
    return lim;
}

void PolygonalMap::Standardize(const double &shift_factor, const double &rounder, Polygons<double> &polygons) {
    auto limits = ComputeLimits(polygons);
    // Make all coordinates positive.
    Shift(-limits.x_min, -limits.y_min, polygons);
    // Shift all coordinates by some small value (important for the map drawer).
    auto shift = shift_factor * std::min(limits.x_max - limits.x_min, limits.y_max - limits.y_min);
    Shift(shift, shift, polygons);
    // Round all coordinates.
    Round(rounder, polygons);
    // Fix the polygons.
    Fix(polygons);
}

draw::MapDrawer PolygonalMap::CreateMapDrawer(const Polygons<double> &polygons) {
    Polygons<double> borders, obstacles;
    for (const auto &poly : polygons) {
        auto clip_poly = clip::Geom2Clipper(poly);
        if (clip::Orientation(clip_poly)) borders.emplace_back(poly.begin(), poly.end());
        else obstacles.emplace_back(poly.begin(), poly.end());
    }
    auto lim = ComputeLimits(polygons);
    return draw::MapDrawer(borders, obstacles, lim.x_max + lim.x_min, lim.y_max + lim.y_min, 125);
}

std::string PolygonalMap::ToString(const Polygon<double> &polygon,
                                   bool parenthesis,
                                   const std::string &delimiter_1,
                                   const std::string &delimiter_2) {
    std::string ret;
    for (const auto &p : polygon) {
        ret += (p == polygon[0]) ? "" : delimiter_2;
        ret += (parenthesis) ? "(" : "";
        ret += p.ToString(delimiter_1);
        ret += (parenthesis) ? ")" : "";
    }
    return ret;
}

std::string PolygonalMap::ToString(const Polygons<double> &polygons,
                                   bool parenthesis,
                                   const std::string &delimiter_1,
                                   const std::string &delimiter_2,
                                   const std::string &delimiter_3) {
    std::string ret;
    for (const auto &poly : polygons) {
        ret += (poly == polygons[0]) ? "" : delimiter_3;
        ret += ToString(poly, parenthesis, delimiter_1, delimiter_2);
    }
    return ret;
}
