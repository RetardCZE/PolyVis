/**
 * File:   vis_region_approx.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/map_coverage/vis_region_approx.h"

#include "trivis/core/geom/generic_geom_utils.h"
#include "trivis/core/utils/clipper_utils.h"

using namespace trivis;
using namespace trivis::map_coverage;
using namespace trivis::core;
using namespace trivis::core::geom;

RadialVisibilityRegion map_coverage::SampleArcEdges(
    const RadialVisibilityRegion &vis_reg,
    double max_sample_beta
) {
    RadialVisibilityRegion ret;
    ret.seed_id = vis_reg.seed_id;
    ret.seed = vis_reg.seed;
    ret.radius = vis_reg.radius;
    int n_v = static_cast<int>(vis_reg.vertices.size());
    if (n_v <= 1) {
        double a_diff = 2 * M_PI;
        int angle_n = static_cast<int>(std::ceil(a_diff / max_sample_beta));
        double angle_delta = a_diff / static_cast<double>(angle_n);
        double alpha = 0;
        for (int j = 0; j < angle_n - 1; ++j) {
            alpha += angle_delta;
            FPoint p = MakePoint(vis_reg.seed.x + vis_reg.radius * std::cos(alpha), vis_reg.seed.y + vis_reg.radius * std::sin(alpha));
            ret.vertices.push_back({-1, -4, p});
        }
        return ret;
    }
    for (int i_prev = n_v - 1, i = 0; i < n_v; i_prev = i++) {
        const auto v_prev = vis_reg.vertices[i_prev];
        const auto v = vis_reg.vertices[i];
        if (v.edge_flag != -2) {
            ret.vertices.push_back(v);
            continue;
        }
        double a0 = std::atan2(v_prev.point.y - vis_reg.seed.y, v_prev.point.x - vis_reg.seed.x);
        double a1 = std::atan2(v.point.y - vis_reg.seed.y, v.point.x - vis_reg.seed.x);
        if (a1 < a0) {
            a1 += 2.0 * M_PI;
        }
        double a_diff = a1 - a0;
        int angle_n = static_cast<int>(std::ceil(a_diff / max_sample_beta));
        double angle_delta = a_diff / static_cast<double>(angle_n);
        double alpha = a0;
        for (int j = 0; j < angle_n - 1; ++j) {
            alpha += angle_delta;
            FPoint p = MakePoint(vis_reg.seed.x + vis_reg.radius * std::cos(alpha), vis_reg.seed.y + vis_reg.radius * std::sin(alpha));
            ret.vertices.push_back({-1, -4, p});
        }
        ret.vertices.push_back(v);
        ret.vertices.back().edge_flag = -4;
    }
    return ret;
}

ClipperLib::Paths map_coverage::ToClipper(
    const PolyMap &map
) {
    ClipperLib::Paths ret;
    const auto &lim = map.limits();
    double normalizer = std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    ret.reserve(1 + map.holes().size());
    ret.push_back(utils::Geom2Clipper(map.border(), normalizer, {lim.x_min, lim.y_min}));
    for (const auto &hole: map.holes()) {
        ret.push_back(utils::Geom2Clipper(hole, normalizer, {lim.x_min, lim.y_min}));
    }
    return ret;
}

ClipperLib::Path map_coverage::ToClipper(const FPolygon &polygon, const FLimits &lim) {
    double normalizer = std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    return utils::Geom2Clipper(polygon, normalizer, {lim.x_min, lim.y_min});
}

ClipperLib::Path map_coverage::ToClipper(
    const RadialVisibilityRegion &vis_reg,
    const FLimits &lim
) {
    FPolygon polygon;
    polygon.reserve(vis_reg.vertices.size());
    for (const auto &v: vis_reg.vertices) {
        polygon.push_back(v.point);
    }
    double normalizer = std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    return utils::Geom2Clipper(polygon, normalizer, {lim.x_min, lim.y_min});
}

FPolygon map_coverage::FromClipper(
    const ClipperLib::Path &polygon,
    const FLimits &lim
) {
    double normalizer = std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    return utils::Clipper2Geom(polygon, normalizer, {lim.x_min, lim.y_min});
}

FPolygons map_coverage::FromClipper(
    const ClipperLib::Paths &polygons,
    const FLimits &lim
) {
    FPolygons ret;
    ret.reserve(polygons.size());
    for (const auto &polygon: polygons) {
        ret.push_back(FromClipper(polygon, lim));
    }
    return ret;
}

void map_coverage::Fix(
    ClipperLib::Path &polygon
) {
    ClipperLib::CleanPolygon(polygon);
    ClipperLib::Paths simple_polygons;
    ClipperLib::SimplifyPolygon(polygon, simple_polygons);
    if (!simple_polygons.empty()) {
        polygon = std::move(simple_polygons[0]);
        double area_max = ClipperLib::Area(polygon);
        for (int i = 1; i < simple_polygons.size(); ++i) {
            double area = ClipperLib::Area(simple_polygons[i]);
            if (area > area_max) {
                area_max = area;
                polygon = std::move(simple_polygons[i]);
            }
        }
    }
}

void map_coverage::Fix(
    ClipperLib::Paths &polygons
) {
    ClipperLib::CleanPolygons(polygons);
    ClipperLib::Paths simple_polygons;
    ClipperLib::SimplifyPolygons(polygons, simple_polygons, ClipperLib::pftNonZero);
    polygons = simple_polygons;
}

void map_coverage::ClipOff(
    const ClipperLib::Path &region,
    ClipperLib::Paths &map
) {
    ClipperLib::Clipper clipper;
    clipper.AddPath(region, ClipperLib::ptClip, true);
    clipper.AddPaths(map, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctDifference, map, ClipperLib::pftNonZero);
    clipper.Clear();
    Fix(map);
}

void map_coverage::ClipOff(
    const ClipperLib::Paths &regions,
    ClipperLib::Paths &map
) {
    ClipperLib::Clipper clipper;
    clipper.AddPaths(regions, ClipperLib::ptClip, true);
    clipper.AddPaths(map, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctDifference, map, ClipperLib::pftNonZero);
    clipper.Clear();
    Fix(map);
}

void map_coverage::ClipOff(
    const std::vector<VisRegWithPolygonApprox> &coverage,
    ClipperLib::Paths &map
) {
    ClipperLib::Clipper clipper;
    for (const auto &cov: coverage) {
        clipper.AddPath(cov.approx_clipper, ClipperLib::ptClip, true);
    }
    clipper.AddPaths(map, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctDifference, map, ClipperLib::pftNonZero);
    clipper.Clear();
    Fix(map);
}

void map_coverage::ClipOff(
    const ClipperLib::Path &region,
    std::vector<ClipperLib::Paths> &map
) {
    ClipperLib::Clipper clipper;
    clipper.AddPath(region, ClipperLib::ptClip, true);
    for (auto &reg: map) {
        clipper.AddPaths(reg, ClipperLib::ptSubject, true);
    }
    ClipperLib::PolyTree sol;
    clipper.Execute(ClipperLib::ctDifference, sol, ClipperLib::pftNonZero);
    clipper.Clear();
    map.clear();
    for (int i = 0; i < sol.ChildCount(); ++i) {
        const ClipperLib::PolyNode *curr = sol.Childs[i];
        const ClipperLib::PolyNode *parent = curr->Parent;
        ClipperLib::Paths aux = {curr->Contour};
        while (curr->GetNext()) {
            curr = curr->GetNext();
            if (curr->Parent == parent) {
                break;
            }
            aux.push_back(curr->Contour);
        }
        Fix(aux);
        std::vector<ClipperLib::Paths> regions;
        ClipperLib::Paths holes;
        for (const auto &reg: aux) {
            if (ClipperLib::Orientation(reg)) {
                regions.push_back({reg});
            } else {
                holes.push_back(reg);
            }
        }
        for (const auto &hole: holes) {
            for (auto &reg: regions) {
                if (ClipperLib::PointInPolygon(hole.front(), reg.front())) {
                    reg.push_back(hole);
                    break;
                }
            }
        }
        for (const auto &polygon: regions) {
            map.push_back(polygon);
        }
    }
}

void map_coverage::ClipOff(
    const ClipperLib::Paths &regions,
    std::vector<ClipperLib::Paths> &map
) {
    ClipperLib::Clipper clipper;
    clipper.AddPaths(regions, ClipperLib::ptClip, true);
    for (auto &reg: map) {
        clipper.AddPaths(reg, ClipperLib::ptSubject, true);
    }
    ClipperLib::PolyTree sol;
    clipper.Execute(ClipperLib::ctDifference, sol, ClipperLib::pftNonZero);
    clipper.Clear();
    map.clear();
    for (int i = 0; i < sol.ChildCount(); ++i) {
        const ClipperLib::PolyNode *curr = sol.Childs[i];
        const ClipperLib::PolyNode *parent = curr->Parent;
        ClipperLib::Paths aux = {curr->Contour};
        while (curr->GetNext()) {
            curr = curr->GetNext();
            if (curr->Parent == parent) {
                break;
            }
            aux.push_back(curr->Contour);
        }
        Fix(aux);
        std::vector<ClipperLib::Paths> polygons;
        ClipperLib::Paths holes;
        for (const auto &reg: aux) {
            if (ClipperLib::Orientation(reg)) {
                polygons.push_back({reg});
            } else {
                holes.push_back(reg);
            }
        }
        for (int j = 0; j < holes.size(); ++j) {
            for (int k = 0; k < polygons.size(); ++k) {
                if (ClipperLib::PointInPolygon(holes[j].front(), polygons[k].front())) {
                    polygons[k].push_back(holes[j]);
                    break;
                }
            }
        }
        for (int j = 0; j < polygons.size(); ++j) {
            map.push_back(polygons[j]);
        }
    }
}
void map_coverage::ClipOff(
    const std::vector<VisRegWithPolygonApprox> &coverage,
    std::vector<ClipperLib::Paths> &map
) {
    ClipperLib::Clipper clipper;
    for (const auto &cov: coverage) {
        clipper.AddPath(cov.approx_clipper, ClipperLib::ptClip, true);
    }
    for (auto &reg: map) {
        clipper.AddPaths(reg, ClipperLib::ptSubject, true);
    }
    ClipperLib::PolyTree sol;
    clipper.Execute(ClipperLib::ctDifference, sol, ClipperLib::pftNonZero);
    clipper.Clear();
    map.clear();
    for (int i = 0; i < sol.ChildCount(); ++i) {
        const ClipperLib::PolyNode *curr = sol.Childs[i];
        const ClipperLib::PolyNode *parent = curr->Parent;
        ClipperLib::Paths aux = {curr->Contour};
        while (curr->GetNext()) {
            curr = curr->GetNext();
            if (curr->Parent == parent) {
                break;
            }
            aux.push_back(curr->Contour);
        }
        Fix(aux);
        std::vector<ClipperLib::Paths> polygons;
        ClipperLib::Paths holes;
        for (const auto &reg: aux) {
            if (ClipperLib::Orientation(reg)) {
                polygons.push_back({reg});
            } else {
                holes.push_back(reg);
            }
        }
        for (int j = 0; j < holes.size(); ++j) {
            for (int k = 0; k < polygons.size(); ++k) {
                if (ClipperLib::PointInPolygon(holes[j].front(), polygons[k].front())) {
                    polygons[k].push_back(holes[j]);
                    break;
                }
            }
        }
        for (int j = 0; j < polygons.size(); ++j) {
            map.push_back(polygons[j]);
        }
    }
}

ClipperLib::Paths map_coverage::Intersection(
    const ClipperLib::Path &region,
    const ClipperLib::Paths &map
) {
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper;
    clipper.AddPath(region, ClipperLib::ptSubject, true);
    clipper.AddPaths(map, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, ret, ClipperLib::pftNonZero);
    clipper.Clear();
    return ret;
}

double map_coverage::ClipperArea(const ClipperLib::Paths &regions) {
    double area = 0.0;
    for (const auto &reg: regions) {
        area += ClipperLib::Area(reg);
    }
    return area;
}

double map_coverage::ClipperArea(const std::vector<ClipperLib::Paths> &regions) {
    double area = 0.0;
    for (const auto &reg: regions) {
        area += ClipperArea(reg);
    }
    return area;
}

VisRegWithPolygonApprox map_coverage::MakeVisRegWithPolygonApprox(
    const FLimits &lim,
    RadialVisibilityRegion vis_reg,
    std::optional<double> max_sample_beta,
    std::optional<double> max_sample_dist
) {
    VisRegWithPolygonApprox ret;
    if (max_sample_beta) {
        ret.max_sample_beta = *max_sample_beta;
        ret.max_sample_dist = ret.max_sample_dist * vis_reg.radius;
    } else if (max_sample_dist) {
        ret.max_sample_dist = *max_sample_dist;
        ret.max_sample_beta = ret.max_sample_dist / vis_reg.radius;
    }
    ret.orig_arcs_approx = SampleArcEdges(vis_reg, (max_sample_beta || max_sample_dist) ? ret.max_sample_beta : M_PI / 36.0);
    ret.approx_clipper = ToClipper(ret.orig_arcs_approx, lim);
    Fix(ret.approx_clipper);
    ret.approx_simple = FromClipper(ret.approx_clipper, lim);
    ret.orig = std::move(vis_reg);
    return ret;
}

