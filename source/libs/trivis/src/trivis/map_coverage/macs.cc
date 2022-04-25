/**
 * File:   macs.cc
 *
 * Date:   07.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/map_coverage/macs.h"

#include <iostream>

#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/geom/intersections.h"

#include "trivis/core/utils/generic_utils.h"

using namespace trivis;
using namespace trivis::core;
using namespace trivis::core::geom;
using namespace trivis::map_coverage;

std::vector<int> map_coverage::ReflexVertices(
    const RadialVisibilityRegion &vis_reg,
    double eps
) {
    std::vector<int> ret;
    const auto &v = vis_reg.vertices;
    if (v.size() > 2) {
        int n_minus_1 = static_cast<int>(v.size()) - 1;
        for (int i_prev = n_minus_1, i = 0, i_next = 1; i < v.size(); i_prev = i++) {
            const auto &vi_prev = v[i_prev];
            const auto &vi = v[i];
            const auto &vi_next = v[i_next];
            if (vi.edge_flag > -2 && vi_next.edge_flag > -2 && TurnsRightWithEps(vi_prev.point, vi.point, vi_next.point, eps)) {
                ret.push_back(i);
            }
            i_next = i_next == n_minus_1 ? 0 : i_next + 1;
        }
    }
    return ret;
}

/**
 * If two intersections are obtained, then the are returned ordered according to direction 'from a to b'.
 */
FPoints ComputeOrderedLineCircleIntersections(
    const FPoint &a,
    const FPoint &b,
    const FPoint &cp,
    double radius,
    double eps
) {
    FPoints ret = LineCircleIntersections(a, b, cp, radius, false, eps);
    if (ret.size() == 2) {
        FPoint preferred_direction = b - a; // direction 'from a to b'
        // preferred_direction = preferred_direction / preferred_direction.Norm();
        FPoint u = ret[1] - ret[0]; // direction 'from ret[0] to ret[1]'
        // u = u / u.Norm();
        if (preferred_direction.SquaredDistanceTo(u) > preferred_direction.SquaredDistanceTo(-u)) {
            std::swap(ret[0], ret[1]);
        }
    }
    return ret;
}

RadialVisibilityRegion CircleIntersectionWithLeftHalfPlane(
    const RadialVisibilityRegion &vis_reg,
    const FPoint &a,
    const FPoint &b,
    double eps
) {
    assert(vis_reg.vertices.empty());
    if (vis_reg.radius <= 0.0) {
        return vis_reg;
    }

    FPoints circ_int = ComputeOrderedLineCircleIntersections(a, b, vis_reg.seed, vis_reg.radius, eps);

    if (circ_int.size() != 2) {
        return vis_reg; // The line is too far away.
    }

    RadialVisibilityRegion res;
    res.seed_id = vis_reg.seed_id;
    res.seed = vis_reg.seed;
    res.radius = vis_reg.radius;
    res.vertices.push_back({-1, -2, circ_int[0]});
    res.vertices.push_back({-1, -3, circ_int[1]});
    return res;
}

bool AddArcOrNot(
    const FPoint &a,
    const FPoint &b,
    const VisibilityRegionVertex &v_prev,
    const VisibilityRegionVertex &v,
    Orientation orient_v_prev,
    Orientation orient_v,
    std::optional<FPoints> &circ_int_opt,
    RadialVisibilityRegion &res,
    double eps
) {
    if (!circ_int_opt) { // Compute LineCircle intersections if not yet computed.
        circ_int_opt = ComputeOrderedLineCircleIntersections(a, b, res.seed, res.radius, eps);
    }

    if (circ_int_opt->size() != 2) {
        return false; // The line is too far away.
    }

    const FPoint &circ_int_0 = (*circ_int_opt)[0];
    const FPoint &circ_int_1 = (*circ_int_opt)[1];

    if (orient_v_prev == Orientation::kLeftTurn) {
        if (orient_v == Orientation::kLeftTurn) {
            ///  === [LL]: v0 ~ left of (a,b), v1 ~ left of (a,b) ====
            if (TurnsRight(v_prev.point, v.point, circ_int_0)) {
                res.vertices.push_back({-1, v.edge_flag, circ_int_0}); // 1st intersection
                res.vertices.push_back({-1, -3, circ_int_1}); // 2nd intersection
                res.vertices.push_back(v);
                return true;
            } else {
                res.vertices.push_back(v);
                return true;
            }
        } else if (orient_v == Orientation::kCollinear) {
            /// === [LC]: v0 ~ left of (a,b), v1 ~ on (a,b) =========
            if (TurnsRight(v_prev.point, v.point, circ_int_0)) {
                res.vertices.push_back({-1, v.edge_flag, circ_int_0}); // 1st intersection
                res.vertices.push_back({v.vertex_flag, -3, v.point}); // 2nd intersection = v1
                return true;
            } else {
                res.vertices.push_back(v);
                return true;
            }
        } else { // Orientation::kRightTurn
            /// === [LR]: v0 ~ left of (a,b), v1 ~ right of (a,b) ===
            res.vertices.push_back({-1, v.edge_flag, circ_int_0}); // 1st intersection
            return true;
        }
    } else if (orient_v_prev == Orientation::kCollinear) {
        if (orient_v == Orientation::kLeftTurn) {
            /// === [CL]: v0 ~ on (a,b), v1 ~ left of (a,b) ==========
            if (TurnsRight(v_prev.point, v.point, circ_int_1)) {
                res.vertices.push_back({-1, -3, circ_int_1}); // 2nd intersection (the 1st one must have been added before)
                res.vertices.push_back(v);
                return true;
            } else {
                res.vertices.push_back(v);
                return true;
            }
        } else if (orient_v == Orientation::kCollinear) {
            /// === [CC]: v0 ~ on (a,b), v1 ~ on (a,b) ===============
            FPoint preferred_direction = b - a;
            FPoint u = v.point - v_prev.point;
            if (preferred_direction.SquaredDistanceTo(u) < preferred_direction.SquaredDistanceTo(-u)) {
                // arc is right to (a,b)
                res.vertices.push_back({v.vertex_flag, -3, v.point});
                return true;
            } else {
                // arc is left to (a,b)
                res.vertices.push_back(v);
                return true;
            }
        } else { // Orientation::kRightTurn
            /// === [CR]: v0 ~ on (a,b), v1 ~ right of (a,b) =========
            if (TurnsRight(v_prev.point, v.point, circ_int_0)) {
                res.vertices.push_back({-1, v.edge_flag, circ_int_0}); // 1st intersection (the 2nd one must have been added before)
                return true;
            } else {
                // DO NOTHING
                return true;
            }
        }
    } else { // Orientation::kRightTurn
        if (orient_v == Orientation::kLeftTurn) {
            /// === [RL]: v0 ~ right of (a,b), v1 ~ left of (a,b) ====
            res.vertices.push_back({-1, -3, circ_int_1}); // 2nd intersection
            res.vertices.push_back(v);
            return true;
        } else if (orient_v == Orientation::kCollinear) {
            /// === [RC]: v0 ~ right of (a,b), v1 ~ on (a,b) =========
            if (TurnsRight(v_prev.point, v.point, circ_int_1)) {
                res.vertices.push_back({-1, -3, circ_int_1}); // 2nd intersection
                res.vertices.push_back(v); // 1st intersection
                return true;
            } else {
                res.vertices.push_back({v.vertex_flag, -3, v.point});
                return true;
            }
        } else { // Orientation::kRightTurn
            /// === [RR]: v0 ~ right of (a,b), v1 ~ right of (a,b) ===
            if (TurnsRight(v_prev.point, v.point, circ_int_0)) {
                res.vertices.push_back({-1, -3, circ_int_1}); // 2nd intersection
                res.vertices.push_back({-1, v.edge_flag, circ_int_0}); // 1st intersection
                return true;
            } else {
                // DO NOTHING
                return true;
            }
        }
    }
}

void AddSegmentOrNot(
    const FPoint &a,
    const FPoint &b,
    const VisibilityRegionVertex &v_prev,
    const VisibilityRegionVertex &v,
    Orientation orient_v0,
    Orientation orient_v1,
    RadialVisibilityRegion &res
) {
    if (orient_v0 == Orientation::kLeftTurn) {
        if (orient_v1 == Orientation::kLeftTurn) {
            ///  === [LL]: v0 ~ left of (a,b), v1 ~ left of (a,b) ====
            res.vertices.push_back(v);
            return;
        } else if (orient_v1 == Orientation::kCollinear) {
            /// === [LC]: v0 ~ left of (a,b), v1 ~ on (a,b) =========
            res.vertices.push_back(v);
            return;
        } else { // Orientation::kRightTurn
            /// === [LR]: v0 ~ left of (a,b), v1 ~ right of (a,b) ===
            FPoint intersection;
            if (LineLineIntersectionNotCollinear(a, b, v_prev.point, v.point, intersection)) {
                res.vertices.push_back({-1, v.edge_flag, intersection});
                return;
            } else {
                res.vertices.push_back(v);
                return;
            }
        }
    } else if (orient_v0 == Orientation::kCollinear) {
        if (orient_v1 == Orientation::kLeftTurn) {
            /// === [CL]: v0 ~ on (a,b), v1 ~ left of (a,b) ==========
            res.vertices.push_back(v);
            return;
        } else if (orient_v1 == Orientation::kCollinear) {
            /// === [CC]: v0 ~ on (a,b), v1 ~ on (a,b) ===============
            res.vertices.push_back(v);
            return;
        } else { // Orientation::kRightTurn
            /// === [CR]: v0 ~ on (a,b), v1 ~ right of (a,b) =========
            // DO NOTHING
            return;
        }
    } else { // Orientation::kRightTurn
        if (orient_v1 == Orientation::kLeftTurn) {
            /// === [RL]: v0 ~ right of (a,b), v1 ~ left of (a,b) ====
            FPoint intersection;
            if (LineLineIntersectionNotCollinear(a, b, v_prev.point, v.point, intersection)) {
                res.vertices.push_back({-1, -3, intersection});
                res.vertices.push_back(v);
                return;
            } else {
                res.vertices.push_back(v_prev);
                res.vertices.push_back(v);
                return;
            }
        } else if (orient_v1 == Orientation::kCollinear) {
            /// === [RC]: v0 ~ right of (a,b), v1 ~ on (a,b) =========
            res.vertices.push_back({v.vertex_flag, -3, v.point});
            return;
        } else { // Orientation::kRightTurn
            /// === [RR]: v0 ~ right of (a,b), v1 ~ right of (a,b) ===
            // DO NOTHING
            return;
        }
    }
}

/**
 * Intersection of the region with the half plane left to the line (b, p2).
 */
[[nodiscard]] RadialVisibilityRegion map_coverage::IntersectionWithLeftHalfPlane(
    const RadialVisibilityRegion &vis_reg,
    const FPoint &a,
    const FPoint &b,
    double eps
) {
    assert(a != b);

    if (vis_reg.vertices.empty()) {
        return CircleIntersectionWithLeftHalfPlane(vis_reg, a, b, eps);
    }

    RadialVisibilityRegion res;
    res.seed_id = vis_reg.seed_id;
    res.seed = vis_reg.seed;
    res.radius = vis_reg.radius;

    std::optional<FPoints> circ_int_opt = std::nullopt;
    ComputeOrderedLineCircleIntersections(a, b, vis_reg.seed, vis_reg.radius, eps);
    const VisibilityRegionVertex *v_prev_ptr = &vis_reg.vertices.back();
    Orientation orient_prev = OrientWithEps(a, b, v_prev_ptr->point, eps);
    for (int i = 0; i < vis_reg.vertices.size(); ++i) {
        const VisibilityRegionVertex &v = vis_reg.vertices[i];
        assert(v.point != v_prev_ptr->point);
        Orientation orient = OrientWithEps(a, b, v.point, eps);
        if (v.edge_flag == -2) {
            if (!AddArcOrNot(a, b, *v_prev_ptr, v, orient_prev, orient, circ_int_opt, res, eps)) {
                // The line is too far away.
                return vis_reg;
            }
        } else {
            AddSegmentOrNot(a, b, *v_prev_ptr, v, orient_prev, orient, res);
        }
        v_prev_ptr = &v;
        orient_prev = orient;
    }
    return res;
}

RadialVisibilityRegion map_coverage::Kernel(
    const RadialVisibilityRegion &vis_reg,
    const std::vector<int> &reflex_vertices,
    double min_edge_length,
    double eps
) {
    if (reflex_vertices.empty()) {
        return vis_reg;
    }
    const int max_id = static_cast<int>(vis_reg.vertices.size()) - 1;
    const int max_reflex_id = static_cast<int>(reflex_vertices.size()) - 1;
    RadialVisibilityRegion res = vis_reg;
    for (int i = 0; i < reflex_vertices.size(); ++i) {
        int id_curr = reflex_vertices[i];
        int id_prev = (id_curr == 0) ? max_id : id_curr - 1;
        // Clip off half-plane given by prev, and curr.
        res = IntersectionWithLeftHalfPlane(res, vis_reg.vertices[id_prev].point, vis_reg.vertices[id_curr].point, eps);
        RemoveShortEdges(min_edge_length, res);
        if (res.vertices.size() <= 1) {
            // Return empty kernel.
            res.radius = 0.0;
            res.vertices.clear();
            return res;
        }
        int id_next = (id_curr == max_id) ? 0 : id_curr + 1;
        int id_reflex_next = (i == max_reflex_id) ? reflex_vertices[0] : reflex_vertices[i + 1];
        if (id_next != id_reflex_next) {
            // Only if the next is not reflex: clip off half-plane given by curr, and next.
            res = IntersectionWithLeftHalfPlane(res, vis_reg.vertices[id_curr].point, vis_reg.vertices[id_next].point, eps);
            RemoveShortEdges(min_edge_length, res);
            if (res.vertices.size() <= 1) {
                // Return empty kernel.
                res.radius = 0.0;
                res.vertices.clear();
                return res;
            }
        }

    }

    return res;
}

double map_coverage::Area(const RadialVisibilityRegion &vis_reg) {
    double radius_sq = vis_reg.radius * vis_reg.radius;
    int n = static_cast<int>(vis_reg.vertices.size());
    if (n <= 1) {
        return M_PI * radius_sq;
    }
    double area = 0;
    for (int i_prev = n - 1, i = 0; i < n; i_prev = i++) {
        const auto &v0 = vis_reg.vertices[i_prev];
        const auto &v1 = vis_reg.vertices[i];
        area += static_cast<double>(v1.point.x + v0.point.x) * static_cast<double>(v1.point.y - v0.point.y) / 2.0;
        if (v1.edge_flag == -2) {
            double a0 = std::atan2(v0.point.y - vis_reg.seed.y, v0.point.x - vis_reg.seed.x);
            double a1 = std::atan2(v1.point.y - vis_reg.seed.y, v1.point.x - vis_reg.seed.x);
            double a_diff = a1 - a0;
            a_diff = std::atan2(std::sin(a_diff), std::cos(a_diff));
            double circular_segment_area = radius_sq * (a_diff - std::sin(a_diff)) / 2.0;
            if (circular_segment_area < 0.0) {
                area += M_PI * radius_sq + circular_segment_area;
            } else {
                area += circular_segment_area;
            }
        }
    }
    return area;
}

FPoint GetBisectorAtVertex(
    int v_id,
    const RadialVisibilityRegion &kernel,
    double eps
) {
    const auto &v_prev = kernel.vertices[v_id == 0 ? kernel.vertices.size() - 1 : v_id - 1];
    const auto &v = kernel.vertices[v_id];
    const auto &v_next = kernel.vertices[v_id + 1 == kernel.vertices.size() ? 0 : v_id + 1];
    FPoint p, a, b;
    if (v.edge_flag == -2) {
        // Edge (v_prev, v) is an arc.
        FPoint u = kernel.seed - v.point;
        a = {u.y, -u.x};
    } else {
        // Edge (v_prev, v) is a segment.
        a = v.point - v_prev.point;
    }
    if (v_next.edge_flag == -2) {
        // Edge (v, v_next) is an arc.
        FPoint u = kernel.seed - v.point;
        b = {-u.y, u.x};
    } else {
        // Edge (v, v_next) is a segment.
        b = v.point - v_next.point;
    }
    a = a / a.Norm();
    b = b / b.Norm();
    p = (a + b) / 2.0;
    if (p.Norm() < eps) {
        p = {a.y, -a.x};
    } else {
        p = p / p.Norm();
    }
    return v.point + p;
}

/**
 * Orders reflex vertices of vis_reg according to their distance to the kernel of vis_reg.
 *
 * Based on https://liris.cnrs.fr/Documents/Liris-1909.pdf,
 * Algorithm in Sec. 3.2 (Algorithms and complexity analysis), heading: Distance-to-kernel computation.
 */
void map_coverage::OrderReflexVertices(
    const RadialVisibilityRegion &vis_reg,
    const RadialVisibilityRegion &kernel,
    std::vector<int> &reflex_vertices,
    std::vector<int> &closest_kernel_edges,
    double eps
) {
    if (reflex_vertices.empty()) {
        return;
    }

    int n_reflex = static_cast<int>(reflex_vertices.size());
    int n_kernel = static_cast<int>(kernel.vertices.size());
    bool kernel_is_just_seed = kernel.radius == 0.0;

    // Find the closest edge ej of kernel to point v0 ∈ vis_reg.
    int ej_v1_id = -1;
    const FPoint &v0 = vis_reg.vertices[reflex_vertices.front()].point;
    FPoints bisectors;
    if (!kernel_is_just_seed) {
        bisectors.reserve(n_kernel);
        // Compute the bisectors.
        for (int v_id = 0; v_id < n_kernel; ++v_id) {
            bisectors.push_back(GetBisectorAtVertex(v_id, kernel, eps));
        }
        // Get the closest edge.
        for (int ex_v0_id = static_cast<int>(n_kernel) - 1, ex_v1_id = 0; ex_v1_id < n_kernel; ex_v0_id = ex_v1_id++) {
            const auto &ex_v0 = kernel.vertices[ex_v0_id];
            const auto &ex_v1 = kernel.vertices[ex_v1_id];
            if (IsPointInCone(v0, ex_v1.point, ex_v0.point, bisectors[ex_v0_id]) && IsPointInCone(v0, bisectors[ex_v1_id], ex_v1.point, ex_v0.point)) {
                ej_v1_id = ex_v1_id;
                break;
            }
        }
    }
    int ej_v1_id_0 = ej_v1_id;
    closest_kernel_edges.resize(n_reflex, -1);
    std::vector<double> reflex_dist_sq(n_reflex, -1.0);
    for (int i = 0; i < n_reflex; ++i) {
        int reflex_id = reflex_vertices[i];
        const auto &reflex_p = vis_reg.vertices[reflex_id].point;
        if (kernel_is_just_seed) {
            reflex_dist_sq[i] = reflex_p.SquaredDistanceTo(kernel.seed);
        } else {
            int ej_v0_id = (ej_v1_id == 0) ? n_kernel - 1 : ej_v1_id - 1;
            while (!IsPointInCone(reflex_p, bisectors[ej_v1_id], kernel.vertices[ej_v1_id].point, kernel.vertices[ej_v0_id].point)) {
                ej_v1_id = (ej_v1_id == 0) ? n_kernel - 1 : ej_v1_id - 1;
                ej_v0_id = (ej_v1_id == 0) ? n_kernel - 1 : ej_v1_id - 1;
                if (ej_v1_id == ej_v1_id_0) {
                    // Prevent infinite loop in case of some exceptional situation.
                    break;
                }
            }
            reflex_dist_sq[i] = PointSegmentSquaredDistance(reflex_p, kernel.vertices[ej_v0_id].point, kernel.vertices[ej_v1_id].point);
            // note: no need to make a difference between segments and arcs for the reflex_dist_sq computation because no arc kernel edge can be the closest to any vis_reg vertex
        }
        closest_kernel_edges[i] = ej_v1_id;
    }
    closest_kernel_edges = utils::SortBy(closest_kernel_edges, reflex_dist_sq);
    reflex_vertices = utils::SortBy(reflex_vertices, reflex_dist_sq);
}

[[nodiscard]] std::vector<std::array<geom::Point<double>, 2>> ChordGuidingPoints(
    int r0_id,
    int r1_id,
    int closest_kernel_edge_to_r0_v1_id,
    const RadialVisibilityRegion &vis_reg,
    const RadialVisibilityRegion &kernel,
    double min_dist
) {

    // Output.
    std::vector<std::array<geom::Point<double>, 2>> chord_guiding_points;

    // Define auxiliaries.
    int n_kernel = static_cast<int>(kernel.vertices.size());
    int n_vis_reg = static_cast<int>(vis_reg.vertices.size());

    // Define significant indices.
    int r0_prev_id = (r0_id == 0) ? n_vis_reg - 1 : r0_id - 1;
    int r0_next_id = (r0_id + 1 == n_vis_reg) ? 0 : r0_id + 1;

    // Define significant points.
    const auto &r0_prev = vis_reg.vertices[r0_prev_id].point;
    const auto &r0 = vis_reg.vertices[r0_id].point;
    const auto &r0_next = vis_reg.vertices[r0_next_id].point;

    bool kernel_is_just_seed = kernel.radius == 0.0;
    double r0_dist_to_seed = r0.DistanceTo(kernel.seed);

    // -- Extremal chords. --
    chord_guiding_points.push_back({r0_prev, r0});
    chord_guiding_points.push_back({r0, r0_next});

    // -- Double pivot chord. --
    if (r1_id >= 0) { // maybe invalid
        int r1_prev_id = (r1_id == 0) ? n_vis_reg - 1 : r1_id - 1;
        int r1_next_id = (r1_id + 1 == n_vis_reg) ? 0 : r1_id + 1;
        if (r1_id != r0_prev_id && r1_id != r0_next_id) {
            int c0_prev_id = r0_prev_id;
            int c1_prev_id = r1_prev_id;
            auto c0_id = r0_id;
            auto c1_id = r1_id;
            int c0_next_id = r0_next_id;
            int c1_next_id = r1_next_id;
            if (r0_id > r1_id) {
                std::swap(c0_prev_id, c1_prev_id);
                std::swap(c0_id, c1_id);
                std::swap(c0_next_id, c1_next_id);
            }
            const auto &c0_prev = vis_reg.vertices[c0_prev_id].point;
            const auto &c0 = vis_reg.vertices[c0_id].point;
            const auto &c0_next = vis_reg.vertices[c0_next_id].point;
            const auto &c1_prev = vis_reg.vertices[c1_prev_id].point;
            const auto &c1 = vis_reg.vertices[c1_id].point;
            const auto &c1_next = vis_reg.vertices[c1_next_id].point;
            Orientation o_010p = Orient(c0, c1, c0_prev);
            Orientation o_011p = Orient(c0, c1, c1_prev);
            if (o_010p == o_011p && o_010p == Orient(c0, c1, c0_next) && o_011p == Orient(c0, c1, c1_next)) {
                chord_guiding_points.push_back({c0, c1});
            }
        }
    }

    // -- Single pivot chord. --
    FPoint u;
    if (kernel_is_just_seed) {
        if (r0_dist_to_seed < min_dist) {
            auto u1 = r0_prev - r0;
            auto u2 = r0_next - r0;
            auto n = ((u1 / u1.Norm()) + (u2 / u2.Norm())) / 2.0;
            u = {-n.y, n.x};
        } else {
            auto n = r0 - kernel.seed;
            n = n / n.Norm();
            u = {-n.y, n.x};
        }
    } else {
        int closest_kernel_edge_to_r0_v0_id = (closest_kernel_edge_to_r0_v1_id == 0) ? n_kernel - 1 : closest_kernel_edge_to_r0_v1_id - 1;
        const auto &v0 = kernel.vertices[closest_kernel_edge_to_r0_v0_id].point;
        const auto &v1 = kernel.vertices[closest_kernel_edge_to_r0_v1_id].point;
        if (r0.DistanceTo(v0) < min_dist) {
            const auto &v2 = kernel.vertices[(closest_kernel_edge_to_r0_v0_id == 0) ? n_kernel - 1 : closest_kernel_edge_to_r0_v0_id - 1].point;
            auto u1 = v0 - v1;
            auto u2 = v0 - v2;
            auto n = ((u1 / u1.Norm()) + (u2 / u2.Norm())) / 2.0;
            u = {-n.y, n.x};
        } else if (r0.DistanceTo(v1) < min_dist) {
            const auto &v2 = kernel.vertices[(closest_kernel_edge_to_r0_v1_id + 1 == n_kernel) ? 0 : closest_kernel_edge_to_r0_v1_id + 1].point;
            auto u1 = v1 - v0;
            auto u2 = v1 - v2;
            auto n = ((u1 / u1.Norm()) + (u2 / u2.Norm())) / 2.0;
            u = {-n.y, n.x};
        } else {
            char closeness_relation = PointSegmentClosenessRelation(r0, v0, v1);
            if (closeness_relation == 'a') {
                auto n = r0 - v0;
                n = n / n.Norm();
                u = {-n.y, n.x};
            } else if (closeness_relation == 'b') {
                auto n = r0 - v1;
                n = n / n.Norm();
                u = {-n.y, n.x};
            } else {
                u = v1 - v0;
                u = u / u.Norm();
            }
        }
    }
    FPoint c0 = r0 - u;
    FPoint c1 = r0 + u;
    if (Orient(c0, c1, r0_prev) == Orient(c0, c1, r0_next)) {
        chord_guiding_points.push_back({c0, c1});
    }

    return chord_guiding_points;
}

RadialVisibilityRegion map_coverage::MaxAreaConvexSubset(
    const RadialVisibilityRegion &vis_reg,
    double min_edge_length,
    double eps,
    const std::function<double(const core::geom::RadialVisibilityRegion &)> &metrics1,
    const std::optional<std::function<double(const core::geom::RadialVisibilityRegion &)>> &metrics2
) {

    // Compute all reflex vertices.
    std::vector<int> reflex_vertices = ReflexVertices(vis_reg, eps);
    int n_reflex = static_cast<int>(reflex_vertices.size());

    // If the input polygon has no reflex vertex (is convex), then return the original polygon.
    if (reflex_vertices.empty()) {
        return vis_reg;
    }
    // Compute the kernel.
    RadialVisibilityRegion kernel = Kernel(vis_reg, reflex_vertices, min_edge_length, eps);
    // Compute ordered set O of reflex vertices.
    std::vector<int> closest_kernel_edges;
    OrderReflexVertices(vis_reg, kernel, reflex_vertices, closest_kernel_edges, eps);
    std::vector<bool> valid_reflex_vertex(n_reflex, true);
    // Start with the full polygon.
    RadialVisibilityRegion res = vis_reg;

    // Extract the first point in the list O and call it r0.
    int r0_id_id = 0;

    // While O is not empty do.
    while (r0_id_id < n_reflex) {

        int r0_id = reflex_vertices[r0_id_id];

        // Extract the first point in list O and call it r1.
        int r1_id = (r0_id_id + 1 == n_reflex) ? -1 : reflex_vertices[r0_id_id + 1];

        // Choose the best chord that maximizes an area of the result.
        RadialVisibilityRegion best_cut;
        std::vector<std::array<FPoint, 2>> chord_points = ChordGuidingPoints(r0_id, r1_id, closest_kernel_edges[r0_id_id], vis_reg, kernel, min_edge_length);
        double best_val1 = std::numeric_limits<double>::lowest();
        double best_val2 = std::numeric_limits<double>::lowest();
        int best_chord_id = 0;
        for (int chord_id = 0; chord_id < chord_points.size(); ++chord_id) {
            const auto &chord = chord_points[chord_id];
            auto temp_reg = IntersectionWithLeftHalfPlane(res, chord[0], chord[1], eps);
            RemoveShortEdges(min_edge_length, temp_reg);
            double val1, val2;
            val1 = metrics1(temp_reg);
            if (metrics2) {
                val2 = (*metrics2)(temp_reg);
            }
            if (val1 > best_val1 || (metrics2 && val1 == best_val1 && val2 > best_val2)) {
                best_val1 = val1;
                if (metrics2) {
                    best_val2 = val2;
                }
                best_chord_id = chord_id;
                best_cut = std::move(temp_reg);
            }
        }
        // Modify the result accordingly.
        res = std::move(best_cut);

        // Update the list O removing reflex points excluded by the chord, and set r0 <- r1.
        valid_reflex_vertex[r0_id_id] = false;
        for (int reflex_id = r0_id_id + 1; reflex_id < n_reflex; ++reflex_id) {
            if (!TurnsLeftWithEps(chord_points[best_chord_id][0], chord_points[best_chord_id][1], vis_reg.vertices[reflex_vertices[reflex_id]].point, eps)) {
                valid_reflex_vertex[reflex_id] = false;
            }
        }
        ++r0_id_id;
        while (r0_id_id < n_reflex && !valid_reflex_vertex[r0_id_id]) {
            ++r0_id_id;
        }
    }

    return res;
}