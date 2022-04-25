/**
 * File:   random_points.h
 *
 * Date:   06.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_MAP_COVERAGE_RANDOM_POINTS_H_
#define TRIVIS_MAP_COVERAGE_RANDOM_POINTS_H_

#include <random>

#include "trivis/core/geom/geom_types.h"

namespace trivis::map_coverage {

[[nodiscard]] core::geom::FPoint UniformRandomPointInTriangle(
    const core::geom::FPolygon &triangle,
    std::mt19937 &rng
);

[[nodiscard]] core::geom::FPoint UniformRandomPointInRandomTriangle(
    const core::geom::FPolygons &triangles,
    const std::vector<double> &accum_areas,
    std::mt19937 &rng
);

[[nodiscard]] core::geom::FPoint UniformRandomPointInRandomTriangle(
    const core::geom::FPolygons &triangles,
    std::vector<double> &accum_areas,
    std::mt19937 &rng
);

[[nodiscard]] inline core::geom::FPoint UniformRandomPointInRandomTriangle(
    const core::geom::FPolygons &triangles,
    std::mt19937 &rng
) {
    std::vector<double> accum_areas;
    return UniformRandomPointInRandomTriangle(triangles, accum_areas, rng);
}

}

#endif //TRIVIS_MAP_COVERAGE_RANDOM_POINTS_H_
