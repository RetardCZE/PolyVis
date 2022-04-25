/**
 * File:   map_coverage.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/map_coverage/map_coverage.h"

#include "trivis/map_coverage/macs.h"
#include "trivis/map_coverage/random_points.h"

#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/triangulation/constrained_delaunay.h"
#include "trivis/core/utils/generic_utils.h"

using namespace trivis;
using namespace trivis::map_coverage;
using namespace trivis::core;
using namespace trivis::core::geom;

RadialVisibilityRegion ComputeVisibilityRegion(
    int node_id,
    const TriVis &vis,
    std::optional<double> vis_radius,
    double eps
) {
    int num_expansions;
    AbstractVisibilityRegion abs_vis_reg;
    vis.ComputeVisibilityRegion(node_id, vis_radius, abs_vis_reg, num_expansions);
    RadialVisibilityRegion vis_reg = vis.ConvertToVisibilityRegion(abs_vis_reg);
    RemoveAntennas(vis_reg);
    if (vis_radius) {
        vis_reg = trivis::core::TriVis::ConvertToRadialVisibilityRegion(*vis_radius, vis_reg);
    }
    RemoveShortEdges(eps, vis_reg);
    return vis_reg;
}

RadialVisibilityRegion ComputeVisibilityRegion(
    const FPoint &p,
    const TriVis &vis,
    std::optional<double> vis_radius,
    double eps
) {
    int num_expansions;
    AbstractVisibilityRegion abs_vis_reg;
    if (!vis.ComputeVisibilityRegion(p, vis_radius, abs_vis_reg, num_expansions)) {
        std::cerr << "[ComputeVisibilityRegion] Visibility region from seed " << p << " could not be found!\n";
        return {};
    }
    RadialVisibilityRegion vis_reg = vis.ConvertToVisibilityRegion(abs_vis_reg);
    RemoveAntennas(vis_reg);
    if (vis_radius) {
        vis_reg = trivis::core::TriVis::ConvertToRadialVisibilityRegion(*vis_radius, vis_reg);
    }
    RemoveShortEdges(eps, vis_reg);
    return vis_reg;
}

void map_coverage::InitializeCoverageWithNodeRegionsMACS(
    const TriVis &vis,
    const std::optional<double> &vis_radius,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
) {
    const auto &mesh = vis.mesh();
    int n_nodes = static_cast<int>(mesh.nodes.size());
    for (int node_id = 0; node_id < n_nodes; ++node_id) {
        RadialVisibilityRegion region = ComputeVisibilityRegion(node_id, vis, vis_radius, eps_collinear);
        const auto &node = mesh.nodes[node_id];
        int e0 = node.edges.back();
        int e1 = node.edges.front();
        const FPoint &p_prev = mesh.point(mesh.edges[e0].nodes[0] == node_id ? mesh.edges[e0].nodes[1] : mesh.edges[e0].nodes[0]);
        const FPoint &p = mesh.point(node_id);
        const FPoint &p_next = mesh.point(mesh.edges[e1].nodes[0] == node_id ? mesh.edges[e1].nodes[1] : mesh.edges[e1].nodes[0]);
        if (TurnsRight(p_prev, p, p_next)) {
            auto region2 = IntersectionWithLeftHalfPlane(region, p, p_next, eps_collinear);
            RemoveShortEdges(eps_collinear, region2);
            region2 = MaxAreaConvexSubset(region2, eps_min_edge_len, eps_collinear, Area);
            coverage.push_back(MakeVisRegWithPolygonApprox(vis.map().limits(), std::move(region2), eps_max_sample_angle));
            region = IntersectionWithLeftHalfPlane(region, p_prev, p, eps_collinear);
            RemoveShortEdges(eps_collinear, region);
        }
        region = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, Area);
        coverage.push_back(MakeVisRegWithPolygonApprox(vis.map().limits(), std::move(region), eps_max_sample_angle));
    }
}

void map_coverage::InitializeCoverageWithEdgeRegionsMACS(
    const TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    bool include_endpoints,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
) {
    const auto &mesh = vis.mesh();
    for (int edge_id = 0; edge_id < mesh.edges.size(); ++edge_id) {
        const auto &edge = mesh.edges[edge_id];
        if (!edge.is_obstacle()) {
            continue;
        }
        auto edge_samples = SampleSegment2D(mesh.point(edge.nodes[0]), mesh.point(edge.nodes[1]), sample_dist, false, include_endpoints);
        if (edge_samples.empty()) {
            continue;
        }
        for (const auto &sample: edge_samples) {
            RadialVisibilityRegion region = ComputeVisibilityRegion(sample, vis, vis_radius, eps_collinear);
            region = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, Area);
            coverage.push_back(MakeVisRegWithPolygonApprox(vis.map().limits(), std::move(region), eps_max_sample_angle));
        }
    }
}

void map_coverage::CoverPolygonsEdgesMACS(
    const FPolygons &polygons,
    const TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    bool include_endpoints,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
) {
    for (const auto &polygon: polygons) {
        for (int j_prev = static_cast<int>(polygon.size()) - 1, j = 0; j < polygon.size(); j_prev = j++) {
            auto edge_samples = SampleSegment2D(polygon[j_prev], polygon[j], sample_dist, false, include_endpoints);
            if (edge_samples.empty()) {
                continue;
            }
            for (const auto &sample: edge_samples) {
                RadialVisibilityRegion region = ComputeVisibilityRegion(sample, vis, vis_radius, eps_collinear);
                region = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, Area);
                coverage.push_back(MakeVisRegWithPolygonApprox(vis.map().limits(), std::move(region), eps_max_sample_angle));
            }
        }
    }
}

void map_coverage::CoverPolygonsMACS(
    const FPolygons &polygons,
    const TriVis &vis,
    const std::optional<double> &vis_radius,
    double sample_dist,
    double eps_collinear,
    double eps_min_edge_len,
    double eps_max_sample_angle,
    std::vector<VisRegWithPolygonApprox> &coverage
) {
    for (const auto &polygon: polygons) {
        auto polygon_samples = SamplePolygon(polygon, sample_dist);
        if (polygon_samples.empty()) {
            continue;
        }
        for (const auto &sample: polygon_samples) {
            RadialVisibilityRegion region = ComputeVisibilityRegion(sample, vis, vis_radius, eps_collinear);
            region = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, Area);
            coverage.push_back(MakeVisRegWithPolygonApprox(vis.map().limits(), std::move(region), eps_max_sample_angle));
        }
    }
}

FPoint RandomPointFromUncoveredRegion(
    const ClipperLib::Paths &uncovered_region,
    const FLimits &lim,
    std::mt19937 &rng
) {
    FPolygons triangles;
    FPolygons uncovered_region_geom = FromClipper(uncovered_region, lim);
    triangulation::TriangulateMapConstrainedDelaunay(uncovered_region_geom, triangles);
    return UniformRandomPointInRandomTriangle(triangles, rng);
}

FPoints GenerateNRandomPointsFromRegion(
    int n,
    const RadialVisibilityRegion &region,
    const FLimits &lim,
    double eps_max_sample_angle,
    std::mt19937 &rng
) {
    FPoints ret(n);
    FPolygons triangles;
    auto region_approx = ToClipper(SampleArcEdges(region, eps_max_sample_angle), lim);
    Fix(region_approx);
    triangulation::TriangulateMapConstrainedDelaunay({FromClipper(region_approx, lim)}, triangles);
    std::vector<double> accum_triangle_areas;
    for (int i = 0; i < n; ++i) {
        FPoint q = UniformRandomPointInRandomTriangle(triangles, accum_triangle_areas, rng);
        ret[i] = std::move(q);
    }
    return ret;
}

ClipperLib::Paths GetReducedUncoveredRegion(
    const ClipperLib::Paths &uncovered_region,
    const FPoint &seed,
    double radius,
    const FLimits &lim
) {
    if (radius <= 0.0 || radius >= std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min)) {
        return uncovered_region;
    }
    double x_min = seed.x - radius;
    double x_max = seed.x + radius;
    double y_min = seed.y - radius;
    double y_max = seed.y + radius;
    return Intersection(ToClipper(FPolygon{{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}}, lim), uncovered_region);
}

ClipperLib::Paths GetReducedUncoveredRegion(
    const ClipperLib::Paths &uncovered_region,
    const RadialVisibilityRegion &reg,
    const FLimits &lim
) {
    double x_min, x_max, y_min, y_max;
    if (reg.vertices.size() <= 1) {
        if (reg.radius <= 0.0) {
            return uncovered_region;
        }
        x_min = reg.seed.x - reg.radius;
        x_max = reg.seed.x + reg.radius;
        y_min = reg.seed.y - reg.radius;
        y_max = reg.seed.y + reg.radius;
    } else {
        x_min = std::numeric_limits<double>::max();
        x_max = std::numeric_limits<double>::lowest();
        y_min = std::numeric_limits<double>::max();
        y_max = std::numeric_limits<double>::lowest();
        bool has_arc = false;
        for (const auto &v: reg.vertices) {
            if (v.point.x < x_min) x_min = v.point.x;
            if (v.point.x > x_max) x_max = v.point.x;
            if (v.point.y < y_min) y_min = v.point.y;
            if (v.point.y > y_max) y_max = v.point.y;
            if (v.edge_flag == -2) {
                has_arc = true;
            }
        }
        if (0.0 < reg.radius && reg.radius < std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min) && has_arc) {
            x_min = std::min(x_min, reg.seed.x - reg.radius);
            x_max = std::max(x_max, reg.seed.x + reg.radius);
            y_min = std::min(y_min, reg.seed.y - reg.radius);
            y_max = std::max(y_max, reg.seed.y + reg.radius);
        }
    }
    return Intersection(ToClipper(FPolygon{{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}}, lim), uncovered_region);
}

bool map_coverage::CoverWithDualSamplingMACS(
    const TriVis &vis,
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
) {
    if (uncovered_region.empty()) {
        return true; // The coverage is complete.
    }
    Fix(uncovered_region);
    bool is_coverage_complete = false;
    const auto &lim = vis.map().limits();
    for (int i = 0; i < stop_max_iter; ++i) {
        FPoint q = RandomPointFromUncoveredRegion(uncovered_region, lim, rng);
        ClipperLib::Paths uncovered_region_reduced = GetReducedUncoveredRegion(uncovered_region, q, vis_radius ? 2.0 * (*vis_radius) : -1.0, lim);
        RadialVisibilityRegion region_q = ComputeVisibilityRegion(q, vis, vis_radius, eps_collinear);
        std::vector<RadialVisibilityRegion> regions = {region_q};
        FPoints random_points = GenerateNRandomPointsFromRegion(dual_samp_iter, region_q, lim, eps_max_sample_angle, rng);
        for (const auto &r: random_points) {
            regions.push_back(ComputeVisibilityRegion(r, vis, vis_radius, eps_collinear));
        }
        VisRegWithPolygonApprox best;
        double best_val1 = std::numeric_limits<double>::lowest();
        double best_val2 = std::numeric_limits<double>::lowest();
        for (const auto &region: regions) {
            VisRegWithPolygonApprox res_i;
            {   // Create i-th convex region.
                auto vis_reg_subset = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, Area);
                res_i = MakeVisRegWithPolygonApprox(lim, std::move(vis_reg_subset), eps_max_sample_angle);
            }
            double val1 = ClipperArea(Intersection(res_i.approx_clipper, uncovered_region_reduced));
            double val2 = Area(res_i.orig);
            if (val1 > best_val1 || (val1 == best_val1 && val2 > best_val2)) {
                best_val1 = val1;
                best_val2 = val2;
                best = std::move(res_i);
            }
        }
        coverage.push_back(best);
        ClipOff(best.approx_clipper, uncovered_region);
        if (uncovered_region.empty()) {
            is_coverage_complete = true;
            break;
        }
        if (ClipperArea(uncovered_region) / total_area < stop_uncoverage_ratio) {
            break;
        }
    }
    return is_coverage_complete;
}

bool map_coverage::CoverWithDualSamplingMCCS(
    const TriVis &vis,
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
) {
    if (uncovered_region.empty()) {
        return true; // The coverage is complete.
    }
    Fix(uncovered_region);
    bool is_coverage_complete = false;
    const auto &lim = vis.map().limits();
    for (int i = 0; i < stop_max_iter; ++i) {
        FPoint q = RandomPointFromUncoveredRegion(uncovered_region, lim, rng);
        ClipperLib::Paths uncovered_region_reduced = GetReducedUncoveredRegion(uncovered_region, q, vis_radius ? 2.0 * *vis_radius : -1.0, lim);
        RadialVisibilityRegion region_q = ComputeVisibilityRegion(q, vis, vis_radius, eps_collinear);
        std::vector<RadialVisibilityRegion> regions = {region_q};
        FPoints random_points = GenerateNRandomPointsFromRegion(dual_samp_iter, region_q, lim, eps_max_sample_angle, rng);
        for (const auto &r: random_points) {
            regions.push_back(ComputeVisibilityRegion(r, vis, vis_radius, eps_collinear));
        }
        VisRegWithPolygonApprox best;
        double best_val1 = std::numeric_limits<double>::lowest();
        double best_val2 = std::numeric_limits<double>::lowest();
        for (const auto &region: regions) {
            ClipperLib::Paths uncovered_region_reduced_more = GetReducedUncoveredRegion(uncovered_region_reduced, region, lim);
            auto IntersectionWithUncoveredRegion = [eps_max_sample_angle, lim, uncovered_region_reduced_more](const core::geom::RadialVisibilityRegion &vis_reg) -> double {
                auto vis_reg_clip = ToClipper(SampleArcEdges(vis_reg, eps_max_sample_angle), lim);
                Fix(vis_reg_clip);
                return ClipperArea(Intersection(vis_reg_clip, uncovered_region_reduced_more));
            };
            VisRegWithPolygonApprox res_i;
            {   // Create i-th convex region.
                auto vis_reg_subset = MaxAreaConvexSubset(region, eps_min_edge_len, eps_collinear, IntersectionWithUncoveredRegion, Area);
                res_i = MakeVisRegWithPolygonApprox(lim, std::move(vis_reg_subset), eps_max_sample_angle);
            }
            double val1 = ClipperArea(Intersection(res_i.approx_clipper, uncovered_region_reduced_more));
            double val2 = Area(res_i.orig);
            if (val1 > best_val1 || (val1 == best_val1 && val2 > best_val2)) {
                best_val1 = val1;
                best_val2 = val2;
                best = std::move(res_i);
            }
        }
        coverage.push_back(best);
        ClipOff(best.approx_clipper, uncovered_region);
        if (uncovered_region.empty()) {
            is_coverage_complete = true;
            break;
        }
        if (ClipperArea(uncovered_region) / total_area < stop_uncoverage_ratio) {
            break;
        }
    }
    return is_coverage_complete;
}

void map_coverage::FilterCoverage(
    const std::optional<double> &vis_radius,
    double initial_uncoverage_ratio,
    double stop_uncoverage_ratio,
    double total_area,
    std::vector<VisRegWithPolygonApprox> &coverage
) {
    double current_uncoverage_ratio = initial_uncoverage_ratio;
    int n = static_cast<int>(coverage.size());
    std::vector<double> areas;
    areas.reserve(n);
    for (const auto &region: coverage) {
        areas.push_back(-Area(region.orig));
    }
    coverage = utils::SortBy(coverage, areas);
    std::vector<int> indices = utils::SortBy(utils::Range(static_cast<int>(n)), areas);
    auto indices_set = std::set<int>(indices.begin(), indices.end());
    ClipperLib::Clipper clipper;
    for (const int i: indices) {
        if (indices_set.find(i) == indices_set.end()) {
            continue;
        }
        const auto &curr_region = coverage[i];
        const auto &curr_polygon = curr_region.approx_clipper;
        ClipperLib::Paths clip = {curr_polygon};
        for (const int j: indices_set) {
            if (j == i) {
                continue;
            }
            if (vis_radius && curr_region.orig.seed.DistanceTo(coverage[j].orig.seed) > 2.0 * *vis_radius) {
                continue;
            }
            clipper.AddPaths(clip, ClipperLib::ptSubject, true);
            clipper.AddPath(coverage[j].approx_clipper, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, clip, ClipperLib::pftNonZero);
            clipper.Clear();
            if (clip.empty()) {
                indices_set.erase(i);
                break;
            }
        }
        if (!clip.empty()) {
            double remainder_ratio = ClipperArea(clip) / total_area;
            if (current_uncoverage_ratio + remainder_ratio <= stop_uncoverage_ratio) {
                current_uncoverage_ratio += remainder_ratio;
                indices_set.erase(i);
            }
        }
    }
    coverage = utils::Select(coverage, std::vector<int>(indices_set.begin(), indices_set.end()));
}
