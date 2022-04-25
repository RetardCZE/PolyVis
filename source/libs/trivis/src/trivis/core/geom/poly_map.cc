/**
 * File:   poly_map.cc
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/geom/poly_map.h"

#include "trivis/core/geom/robust_geometry.h"

using namespace trivis::core;
using namespace trivis::core::geom;

void PolyMap::RemoveDuplicatePoints() {
    _border = geom::RemoveDuplicatePoints(_border);
    for (auto &hole: _holes) {
        hole = geom::RemoveDuplicatePoints(hole);
    }
}

void PolyMap::RemoveCollinearPoints() {
    _border = geom::RemoveCollinearPoints(_border);
    for (auto &hole: _holes) {
        hole = geom::RemoveCollinearPoints(hole);
    }
}

void PolyMap::ShiftToOrigin() {
    for (auto &p: _border) {
        p.x -= _limits.x_min;
        p.y -= _limits.y_min;
    }
    for (auto &polygon: _holes) {
        for (auto &p: polygon) {
            p.x -= _limits.x_min;
            p.y -= _limits.y_min;
        }
    }
    _limits.x_max -= _limits.x_min;
    _limits.y_max -= _limits.y_min;
    _limits.x_min = 0.0;
    _limits.y_min = 0.0;
}
FPoints PolyMap::ToPoints() const {
    FPoints ret;
    for (const auto &p: _border) {
        ret.push_back(p);
    }
    for (const auto &hole: _holes) {
        for (const auto &p: hole) {
            ret.push_back(p);
        }
    }
    return ret;
}
