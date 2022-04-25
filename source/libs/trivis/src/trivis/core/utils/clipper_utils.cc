/**
 * File:   clipper_utils.cc
 *
 * Date:   28.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/utils/clipper_utils.h"

using namespace trivis::core;
using namespace trivis::core::geom;
using namespace trivis::core::utils;

ClipperLib::Path utils::Geom2Clipper(
    const FPolygon &polygon,
    double normalizer,
    const FPoint &subtrahend,
    double multiplier
) {
    ClipperLib::Path ret;
    ret.reserve(polygon.size());
    for (const auto &p: polygon) {
        ret.push_back(Geom2Clipper(p, normalizer, subtrahend, multiplier));
    }
    return ret;
}

FPolygon utils::Clipper2Geom(
    const ClipperLib::Path &polygon,
    double normalizer,
    const FPoint &subtrahend,
    double multiplier
) {
    FPolygon ret;
    ret.reserve(polygon.size());
    for (const auto &p: polygon) {
        ret.push_back(Clipper2Geom(p, normalizer, subtrahend, multiplier));
    }
    return ret;
}

ClipperLib::Paths utils::Geom2Clipper(
    const FPolygons &polygons,
    double normalizer,
    const FPoint &subtrahend,
    double multiplier
) {
    ClipperLib::Paths ret;
    ret.reserve(polygons.size());
    for (const auto &polygon: polygons) {
        ret.push_back(Geom2Clipper(polygon, normalizer, subtrahend, multiplier));
    }
    return ret;
}

FPolygons utils::Clipper2Geom(
    const ClipperLib::Paths &polygons,
    double normalizer,
    const FPoint &subtrahend,
    double multiplier
) {
    FPolygons ret;
    ret.reserve(polygons.size());
    for (const auto &polygon: polygons) {
        ret.push_back(Clipper2Geom(polygon, normalizer, subtrahend, multiplier));
    }
    return ret;
}
