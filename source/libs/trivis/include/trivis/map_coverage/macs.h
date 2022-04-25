/**
 * File:   macs.h
 *
 * Date:   07.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_MAP_COVERAGE_MACS_H_
#define TRIVIS_MAP_COVERAGE_MACS_H_

#include "trivis/core/geom/visibility_region.h"

#include <functional>

#include "trivis/map_coverage/vis_region_approx.h"

namespace trivis::map_coverage {

[[nodiscard]] std::vector<int> ReflexVertices(
    const core::geom::RadialVisibilityRegion &vis_reg,
    double eps
);

[[nodiscard]] core::geom::RadialVisibilityRegion IntersectionWithLeftHalfPlane(
    const core::geom::RadialVisibilityRegion &vis_reg,
    const core::geom::FPoint &a,
    const core::geom::FPoint &b,
    double eps
);

[[nodiscard]] core::geom::RadialVisibilityRegion Kernel(
    const core::geom::RadialVisibilityRegion &vis_reg,
    const std::vector<int> &reflex_vertices,
    double min_edge_length,
    double eps
);

void OrderReflexVertices(
    const core::geom::RadialVisibilityRegion &vis_reg,
    const core::geom::RadialVisibilityRegion &kernel,
    std::vector<int> &reflex_vertices,
    std::vector<int> &closest_kernel_edges,
    double eps
);

double Area(const core::geom::RadialVisibilityRegion &vis_reg);

[[nodiscard]] core::geom::RadialVisibilityRegion MaxAreaConvexSubset(
    const core::geom::RadialVisibilityRegion &vis_reg,
    double min_edge_length,
    double eps,
    const std::function<double(const core::geom::RadialVisibilityRegion &)> &metrics1 = Area,
    const std::optional<std::function<double(const core::geom::RadialVisibilityRegion &)>> &metrics2 = std::nullopt
);

}

#endif //TRIVIS_MAP_COVERAGE_MACS_H_
