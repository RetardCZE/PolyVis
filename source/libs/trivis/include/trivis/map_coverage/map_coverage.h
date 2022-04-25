/**
 * File:   map_coverage.h
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_MAP_COVERAGE_MAP_COVERAGE_H_
#define TRIVIS_MAP_COVERAGE_MAP_COVERAGE_H_

#include <random>

#include "trivis/core/tri_vis.h"

#include "trivis/map_coverage/vis_region_approx.h"

namespace trivis::map_coverage {

void InitializeCoverageWithNodeRegionsMACS(
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
);

void InitializeCoverageWithEdgeRegionsMACS(
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    bool include_endpoints,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
);

void CoverPolygonsEdgesMACS(
    const core::geom::FPolygons &polygons,
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    bool include_endpoints,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
);

void CoverPolygonsMACS(
    const core::geom::FPolygons &polygons,
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
);

bool CoverWithDualSamplingMACS(
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    int dual_samp_iter,
    int stop_max_iter,
    double stop_uncoverage_ratio,
    double total_area,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage,
    ClipperLib::Paths &uncovered_region,
    std::mt19937 &rng
);

bool CoverWithDualSamplingMCCS(
    const core::TriVis &vis,
    const std::optional<double> &vis_radius,
    int dual_samp_iter,
    int stop_max_iter,
    double stop_uncoverage_ratio,
    double total_area,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage,
    ClipperLib::Paths &uncovered_region,
    std::mt19937 &rng
);

void FilterCoverage(
    const std::optional<double> &vis_radius,
    double initial_uncoverage_ratio,
    double stop_uncoverage_ratio,
    double total_area,
    std::vector<VisRegWithPolygonApprox> &coverage
);

}

#endif //TRIVIS_MAP_COVERAGE_MAP_COVERAGE_H_
