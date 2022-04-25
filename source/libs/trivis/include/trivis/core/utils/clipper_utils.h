/**
 * File:   clipper_utils.h
 *
 * Date:   28.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_UTILS_CLIPPER_UTILS_H_
#define TRIVIS_CORE_UTILS_CLIPPER_UTILS_H_

#include "trivis/core/libs/clipper/clipper.hpp"
#include "trivis/core/geom/geom_types.h"

namespace trivis::core::utils {

static constexpr double kDefaultMultiplier = 1073741824.0; // 2^30
static constexpr double kDefaultNormalizer = 1.0;

inline double Radius2Clipper(
    double radius,
    double normalizer = kDefaultNormalizer,
    double multiplier = kDefaultMultiplier
) {
    return multiplier * (radius / normalizer);
}

inline ClipperLib::IntPoint Geom2Clipper(
    const geom::FPoint &p,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
) {
    return {static_cast<ClipperLib::cInt>(multiplier * ((p.x - subtrahend.x) / normalizer)), static_cast<ClipperLib::cInt>(multiplier * ((p.y - subtrahend.y) / normalizer))};
}

inline geom::FPoint Clipper2Geom(
    const ClipperLib::IntPoint &p,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
) {
    return {(static_cast<double>(p.X) / multiplier) * normalizer + subtrahend.x, (static_cast<double>(p.Y) / multiplier) * normalizer + subtrahend.y};
}

ClipperLib::Path Geom2Clipper(
    const geom::FPolygon &polygon,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
);

geom::FPolygon Clipper2Geom(
    const ClipperLib::Path &polygon,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
);


ClipperLib::Paths Geom2Clipper(
    const geom::FPolygons &polygons,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
);

geom::FPolygons Clipper2Geom(
    const ClipperLib::Paths &polygons,
    double normalizer = kDefaultNormalizer,
    const geom::FPoint &subtrahend = {0.0, 0.0},
    double multiplier = kDefaultMultiplier
);


}

#endif //TRIVIS_CORE_UTILS_CLIPPER_UTILS_H_
