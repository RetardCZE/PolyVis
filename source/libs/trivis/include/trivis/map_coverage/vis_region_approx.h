/**
 * File:   vis_region_approx.h
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_MAP_COVERAGE_VIS_REGION_APPROX_H_
#define TRIVIS_MAP_COVERAGE_VIS_REGION_APPROX_H_

#include "trivis/core/tri_vis.h"

#include "trivis/core/libs/clipper/clipper.hpp"

namespace trivis::map_coverage {

struct VisRegWithPolygonApprox {
    // Parameters used when converting arcs to polylines.
    double max_sample_beta;
    double max_sample_dist;
    // Original visibility region.
    core::geom::RadialVisibilityRegion orig;
    // The same as above, but arcs are converted to polylines.
    core::geom::RadialVisibilityRegion orig_arcs_approx;
    // The above converted to clipper representation and simplified.
    ClipperLib::Path approx_clipper;
    // The above converted to floating-point polygon.
    core::geom::FPolygon approx_simple;
};

core::geom::RadialVisibilityRegion SampleArcEdges(
    const core::geom::RadialVisibilityRegion &vis_reg,
    double max_sample_beta
);

ClipperLib::Paths ToClipper(
    const core::geom::PolyMap &map
);

ClipperLib::Path ToClipper(
    const core::geom::FPolygon &polygon,
    const core::geom::FLimits &lim
);

ClipperLib::Path ToClipper(
    const core::geom::RadialVisibilityRegion &vis_reg,
    const core::geom::FLimits &lim
);

core::geom::FPolygon FromClipper(
    const ClipperLib::Path &polygon,
    const core::geom::FLimits &lim
);

core::geom::FPolygons FromClipper(
    const ClipperLib::Paths &polygons,
    const core::geom::FLimits &lim
);

void Fix(
    ClipperLib::Path &polygon
);

void Fix(
    ClipperLib::Paths &polygons
);

void ClipOff(
    const ClipperLib::Path &region,
    ClipperLib::Paths &map
);

void ClipOff(
    const ClipperLib::Paths &regions,
    ClipperLib::Paths &map
);

void ClipOff(
    const std::vector<VisRegWithPolygonApprox> &coverage,
    ClipperLib::Paths &map
);

void ClipOff(
    const ClipperLib::Path &region,
    std::vector<ClipperLib::Paths> &map
);

void ClipOff(
    const ClipperLib::Paths &regions,
    std::vector<ClipperLib::Paths> &map
);

void ClipOff(
    const std::vector<VisRegWithPolygonApprox> &coverage,
    std::vector<ClipperLib::Paths> &map
);

ClipperLib::Paths Intersection(
    const ClipperLib::Path &region,
    const ClipperLib::Paths &map
);

double ClipperArea(
    const ClipperLib::Paths &regions
);

double ClipperArea(
    const std::vector<ClipperLib::Paths> &regions
);

VisRegWithPolygonApprox MakeVisRegWithPolygonApprox(
    const core::geom::FLimits &lim,
    core::geom::RadialVisibilityRegion vis_reg,
    std::optional<double> max_sample_beta = M_PI / 36.0,
    std::optional<double> max_sample_dist = std::nullopt
);

}

#endif //TRIVIS_MAP_COVERAGE_VIS_REGION_APPROX_H_
