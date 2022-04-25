/**
 * File:   trivis.cc
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/tri_vis.h"

#include "trivis/core/triangulation/triangulations.h"

#include "trivis/core/geom/generic_geom_utils.h"
#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/geom/intersections.h"

using namespace trivis::core;
using namespace trivis::core::geom;

void TriVis::SetMap(PolyMap map) {
    ClearStructures();
    _bucketing.Init(map.limits());
    _map = std::move(map);
}

void TriVis::TriangulateMapConstrainedDelaunay() {
    triangulation::TriangulateMapConstrainedDelaunay(_map, _mesh, _triangles);
}

void TriVis::FillBucketing() {
    _bucketing.FillBuckets(_triangles);
}

void TriVis::OptimizeBuckets() {
    _bucketing.RemoveDuplicateBucketTriangles();
    _bucketing.OrderBucketTrianglesByIntersectionArea(_triangles);
}

RadialVisibilityRegion TriVis::ConvertToVisibilityRegion(
    const AbstractVisibilityRegion &abstract,
    bool fast_mode
) const {
    RadialVisibilityRegion ret;
    ret.seed_id = abstract.seed_id;
    ret.seed = abstract.seed;
    int n_minus_1 = static_cast<int>(abstract.segments.size()) - 1;
    for (int i_prev = n_minus_1, i = 0; i < abstract.segments.size(); i_prev = i++) {
        AbstractVisibilityRegionSegment seg_prev = abstract.segments[i_prev];
        AbstractVisibilityRegionSegment seg = abstract.segments[i];
        if (seg_prev.v2.is_intersection || seg_prev.v2.id != seg.v1.id) {
            // Append v1.
            AppendNotCollinearWithIntersection(abstract.seed, seg.v1, -1, false, fast_mode, ret);
        }
        // Append v2.
        AppendNotCollinearWithIntersection(abstract.seed, seg.v2, seg.id, i == n_minus_1, fast_mode, ret);
    }
    return ret;
}

RadialVisibilityRegion TriVis::ConvertToRadialVisibilityRegion(
    double radius,
    const RadialVisibilityRegion &visibility_region
) {
    RadialVisibilityRegion ret;
    ret.radius = radius;
    ret.seed_id = visibility_region.seed_id;
    ret.seed = visibility_region.seed;
    double sq_radius = radius * radius;
    int n_minus_1 = static_cast<int>(visibility_region.vertices.size()) - 1;
    for (int i_prev = n_minus_1, i = 0; i < visibility_region.vertices.size(); i_prev = i++) {
        const auto &vi_prev = visibility_region.vertices[i_prev];
        const auto &vi = visibility_region.vertices[i];
        if (vi.edge_flag < -1) {
            // The whole edge is outside the radius: IGNORE IT.
            continue;
        }
        bool is_inside_pi_prev = vi_prev.point.SquaredDistanceTo(visibility_region.seed) <= sq_radius;
        bool is_inside_pi = vi.point.SquaredDistanceTo(visibility_region.seed) <= sq_radius;
        if (is_inside_pi_prev) {
            if (is_inside_pi) {
                // The whole edge is inside the radius (no intersection).
                if (vi_prev.edge_flag < -1) {
                    ret.vertices.push_back({vi_prev.vertex_flag, -2, vi_prev.point});
                }
                ret.vertices.push_back({vi.vertex_flag, vi.edge_flag, vi.point});
            } else {
                // The first endpoint is inside, the second is outside (1 intersection).
                FPoint intersection;
                if (vi.edge_flag < 0) {
                    intersection = vi_prev.point - visibility_region.seed;
                    intersection = intersection / intersection.Norm();
                    intersection = visibility_region.seed + intersection * radius;
                } else {
                    auto intersections = LineCircleIntersections(vi_prev.point, vi.point, visibility_region.seed, radius, true);
                    if (intersections.empty()) {
                        std::cerr << std::fixed << "[ConvertToRadialVisibilityRegion] LineCircle intersections should not be empty (1)! ";
                        std::cerr << "Line: " << vi_prev.point << ", " << vi.point << ", Circle:" << visibility_region.seed << ", " << radius << ".\n";
                        continue;
                    }
                    intersection = intersections[0];
                }
                ret.vertices.push_back({-1, vi.edge_flag, intersection});
            }
        } else if (is_inside_pi) {
            // The first endpoint is outside, the second is inside (1 intersection).
            FPoint intersection;
            if (vi.edge_flag < 0) {
                intersection = vi_prev.point - visibility_region.seed;
                intersection = intersection / intersection.Norm();
                intersection = visibility_region.seed + intersection * radius;
            } else {
                auto intersections = LineCircleIntersections(vi_prev.point, vi.point, visibility_region.seed, radius, true);
                if (intersections.empty()) {
                    std::cerr << std::fixed << "[ConvertToRadialVisibilityRegion] LineCircle intersections should not be empty (2)! ";
                    std::cerr << "Line: " << vi_prev.point << ", " << vi.point << ", Circle:" << visibility_region.seed << ", " << radius << ".\n";
                    continue;
                }
                intersection = intersections[0];
            }
            ret.vertices.push_back({-1, -2, intersection});
            ret.vertices.push_back(vi);
        } else {
            // Both endpoints are outside (2 intersections).
            auto intersections = LineCircleIntersections(vi_prev.point, vi.point, visibility_region.seed, radius, true);
            if (!intersections.empty()) {
                if (intersections.size() > 1 && vi_prev.point.SquaredDistanceTo(intersections[0]) > vi_prev.point.SquaredDistanceTo(intersections[1])) {
                    std::swap(intersections[0], intersections[1]);
                }
                ret.vertices.push_back({-1, -2, intersections[0]});
                if (intersections.size() > 1) {
                    ret.vertices.push_back({-1, vi.edge_flag, intersections[1]});
                }
            }
        }
    }
    return ret;
}

geom::FPolygon TriVis::ConvertToPolygon(
    const RadialVisibilityRegion &visibility_region
) {
    geom::FPolygon ret;
    ret.reserve(visibility_region.vertices.size());
    for (const auto &v: visibility_region.vertices) {
        ret.push_back(v.point);
    }
    return ret;
}

bool TriVis::ComputeVisibilityRegion(
    const FPoint &q,
    std::optional<double> radius,
    AbstractVisibilityRegion &visibility_region,
    int &num_expansions
) const {

    num_expansions = 0;

    // Find triangle where q is located.
    std::optional<int> tri_id_opt = _bucketing.FindTriangle(q, _triangles);
    if (!tri_id_opt) {
        // Return false in case the triangle could not be found.
        return false;
    }
    int tri_id = *tri_id_opt;

    // Use the node version in case q is a node of the triangle.
    const auto &tri = _mesh.triangles[tri_id];
    for (int node_id: tri.nodes) {
        if (q == _mesh.point(node_id)) {
            ComputeVisibilityRegion(node_id, radius, visibility_region, num_expansions);
            return true;
        }
    }

    // Compute radius squared (or set to -1 in case it is not given).
    double sq_radius = (radius && radius > 0.0) ? *radius * *radius : -1.0;

    // Save the seed.
    visibility_region.seed_id = -1; // seed is NOT one of mesh's nodes
    visibility_region.seed = q;

    // Expand edges of the triangle.
    for (auto edge_id: _mesh.triangles[tri_id].edges) {
        const auto &edge = _mesh.edges[edge_id];
        int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

        // Establish the left and right restriction points.
        int rest_l_id = edge.nodes[0];
        int rest_r_id = edge.nodes[1];
        if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(rest_r_id), _mesh.point(rest_l_id))) {
            std::swap(rest_l_id, rest_r_id);
        }

        // Expand the edge.
        ExpandEdgeVisibilityRegion(1, q, rest_l_id, rest_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, sq_radius, visibility_region, num_expansions);

    }

    return true;
}

void TriVis::ComputeVisibilityRegion(
    int node_id,
    std::optional<double> radius,
    AbstractVisibilityRegion &visibility_region,
    int &num_expansions
) const {

    num_expansions = 0;

    // Compute radius squared (or set to -1 in case it is not given).
    double sq_radius = (radius && radius > 0.0) ? *radius * *radius : -1.0;

    const auto &node = _mesh.nodes[node_id];
    const auto &node_p = node.point;

    // Save the seed.
    visibility_region.seed_id = node_id; // seed is one of mesh's nodes
    visibility_region.seed = node_p;

    // Add the first edge.
    int e_id_1st = node.edges.front();
    const auto &e_1st = _mesh.edges[e_id_1st];
    AbstractVisibilityRegionSegment seg_1st;
    int v_l_id_1st = e_1st.nodes[0];
    int v_r_id_1st = e_1st.nodes[1];
    if (_mesh.nodes[v_l_id_1st].edges.front() == e_id_1st) {
        std::swap(v_l_id_1st, v_r_id_1st);
    }
    seg_1st.id = e_id_1st;
    seg_1st.v1.id = v_r_id_1st;
    seg_1st.v2.id = v_l_id_1st;

    visibility_region.segments.push_back(seg_1st);

    for (int tri_id: node.triangles) {
        const auto &tri = _mesh.triangles[tri_id];
        for (int edge_id: tri.edges) {

            const auto &edge = _mesh.edges[edge_id];
            if (edge.nodes[0] != node_id && edge.nodes[1] != node_id) {
                int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

                // Establish the left and right restriction points.
                int rest_l_id = edge.nodes[0];
                int rest_r_id = edge.nodes[1];
                if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(rest_r_id), _mesh.point(rest_l_id))) {
                    std::swap(rest_l_id, rest_r_id);
                }

                // Expand the edge.
                ExpandEdgeVisibilityRegion(1, node_p, rest_l_id, rest_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, sq_radius, visibility_region, num_expansions);

                break;
            }
        }

    }

    // Add the last edge.
    int e_id_lst = node.edges.back();
    const auto &e_lst = _mesh.edges[e_id_lst];
    AbstractVisibilityRegionSegment seg_lst;
    int v_l_id_lst = e_lst.nodes[0];
    int v_r_id_lst = e_lst.nodes[1];
    if (_mesh.nodes[v_l_id_lst].edges.front() == e_id_lst) {
        std::swap(v_l_id_lst, v_r_id_lst);
    }
    seg_lst.id = e_id_lst;
    seg_lst.v1.id = v_r_id_lst;
    seg_lst.v2.id = v_l_id_lst;

    visibility_region.segments.push_back(seg_lst);

}

void TriVis::ExpandEdgeVisibilityRegion(
    int level,
    const FPoint &q,
    int rest_l_id,
    int rest_r_id,
    int curr_edge_id,
    int curr_edge_tri_id,
    double sq_radius,
    AbstractVisibilityRegion &visibility_region,
    int &num_expansions
) const {

    // current edge (struct)
    const auto &curr_edge = _mesh.edges[curr_edge_id];

    bool too_far_away = sq_radius >= 0.0 && PointSegmentSquaredDistance(q, _mesh.point(curr_edge.nodes[0]), _mesh.point(curr_edge.nodes[1])) >= sq_radius;

    if (too_far_away || curr_edge.is_obstacle()) {
        // Report the edge and return.

        AbstractVisibilityRegionSegment seg;
        seg.id = curr_edge_id;
        if (too_far_away) {
            seg.id = -10 - seg.id;
        }

        // Find out which one of the edge endpoints is 'left' and which one is 'right'.
        int v_l_id = curr_edge.nodes[0];
        int v_r_id = curr_edge.nodes[1];
        if (_mesh.nodes[v_l_id].edges.front() == curr_edge_id) {
            std::swap(v_l_id, v_r_id);
        }

        // Report the first (=right) point.
        if (v_r_id == rest_r_id) {
            seg.v1.id = v_r_id;
        } else {
            // The first point is an intersection of ray (q, right restriction point) and the edge.
            seg.v1.is_intersection = true;
            seg.v1.id_b = rest_r_id;
            seg.v1.id_c = v_l_id;
            seg.v1.id_d = v_r_id;
        }

        // Report the second (=left) point.
        if (v_l_id == rest_l_id) {
            seg.v2.id = v_l_id;
        } else {
            // The second point is an intersection of ray (q, left restriction point) and the edge.
            seg.v2.is_intersection = true;
            seg.v2.id_b = rest_l_id;
            seg.v2.id_c = v_l_id;
            seg.v2.id_d = v_r_id;
        }

        visibility_region.segments.push_back(seg);
        return;
    }

    // Record proper expansion (the expansion is not proper if the current edge is an obstacle or too far away).
    ++num_expansions;

    // Expand neighboring edges.

    // current left restriction point
    const auto &rest_l_p = _mesh.point(rest_l_id);
    // current right restriction point
    const auto &rest_r_p = _mesh.point(rest_r_id);

    // current triangle index
    int tri_id = curr_edge.triangles[curr_edge_tri_id];
    // current triangle (struct)
    const auto &tri = _mesh.triangles[tri_id];
    // node (its index) opposite to the current edge in current triangle
    int opp_id = curr_edge.opposites[curr_edge_tri_id];
    // point corresponding to the opposite node
    const auto &opp_p = _mesh.point(opp_id);
    // index (0, 1, or 2) of the current edge for member 'edges' of the current triangle
    int tri_edge_id = tri.edges[0] == curr_edge_id ? 0 : (tri.edges[1] == curr_edge_id ? 1 : 2);

    // index of the edge on right of the current edge
    int edge_r_id = tri.edges[(tri_edge_id + 1) % 3];
    // edge (struct) on right of the current edge
    const auto &edge_r = _mesh.edges[edge_r_id];
    // index of the other node (not opp) that makes edge_r
    int edge_r_not_opp_id = edge_r.nodes[edge_r.nodes[0] == opp_id ? 1 : 0];
    // true if the edge is obstacle
    bool edge_r_is_obstacle = edge_r.is_obstacle();

    // index of the edge on left of the current edge
    int edge_l_id = tri.edges[(tri_edge_id + 2) % 3];
    // edge (struct) on left of the current edge
    const auto &edge_l = _mesh.edges[edge_l_id];
    // index of the other node (not opp) that makes edge_l
    int edge_l_not_opp_id = edge_l.nodes[edge_l.nodes[0] == opp_id ? 1 : 0];
    // true if the edge is obstacle
    bool edge_l_is_obstacle = edge_l.is_obstacle();

    // true iff the opp_p satisfies restriction given by the left point
    Orientation o_q_rest_l_opp = Orient(q, rest_l_p, opp_p);
    // true iff the opp_p satisfies restriction given by the right point
    Orientation o_q_rest_r_opp = Orient(q, rest_r_p, opp_p);

    // Now decide if the other edges (left=l and right=r of the current edge) of the current triangle will be expanded.

    if (o_q_rest_r_opp != Orientation::kRightTurn) { // the right edge is at least partially visible
        int new_rest_l_id = rest_l_id;
        if (o_q_rest_l_opp != Orientation::kLeftTurn) {
            // update left restriction point if opp is more or the same way restricting
            new_rest_l_id = opp_id;
        }
        int new_edge_tri_id = edge_r.triangles[0] == tri_id ? 1 : 0;

        // *** note: right restriction node will always stay the same

        ExpandEdgeVisibilityRegion(level + 1, q, new_rest_l_id, rest_r_id, edge_r_id, new_edge_tri_id, sq_radius, visibility_region, num_expansions);
    }

    if (o_q_rest_l_opp != Orientation::kLeftTurn) { // the left edge is at least partially visible
        int new_rest_r_id = rest_r_id;
        if (o_q_rest_r_opp != Orientation::kRightTurn) {
            // update right restriction point if opp is more or the same way restricting
            new_rest_r_id = opp_id;
        }
        int new_edge_tri_id = edge_l.triangles[0] == tri_id ? 1 : 0;

        // *** note: left restriction node will always stay the same

        ExpandEdgeVisibilityRegion(level + 1, q, rest_l_id, new_rest_r_id, edge_l_id, new_edge_tri_id, sq_radius, visibility_region, num_expansions);
    }
}

bool TriVis::ComputeVisibleNodes(
    const FPoint &q,
    std::optional<double> radius,
    std::vector<int> &visible_nodes,
    int &num_expansions
) const {

    num_expansions = 0;

    // Find triangle where q is located.
    std::optional<int> tri_id_opt = _bucketing.FindTriangle(q, _triangles);
    if (!tri_id_opt) {
        // Return false in case the triangle could not be found.
        return false;
    }
    int tri_id = *tri_id_opt;

    // Use the node version in case q is a node of the triangle.
    const auto &tri = _mesh.triangles[tri_id];
    for (int node_id: tri.nodes) {
        if (q == _mesh.point(node_id)) {
            ComputeVisibleNodes(node_id, radius, visible_nodes, num_expansions);
            return true;
        }
    }

    // Compute radius squared (or set to -1 in case it is not given).
    double sq_radius = (radius && radius > 0.0) ? *radius * *radius : -1.0;

    // Expand edges of the triangle.
    for (auto edge_id: _mesh.triangles[tri_id].edges) {
        const auto &edge = _mesh.edges[edge_id];
        int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

        // Establish the left and right restriction points.
        int rest_l_id = edge.nodes[0];
        int rest_r_id = edge.nodes[1];
        if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(rest_r_id), _mesh.point(rest_l_id))) {
            std::swap(rest_l_id, rest_r_id);
        }

        // Expand the edge.
        ExpandEdgeVisibleNodes(1, q, rest_l_id, rest_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, sq_radius, visible_nodes, num_expansions);

    }

    return true;
}

void TriVis::ComputeVisibleNodes(
    int node_id,
    std::optional<double> radius,
    std::vector<int> &visible_nodes,
    int &num_expansions
) const {
    num_expansions = 0;

    // Compute radius squared (or set to -1 in case it is not given).
    double sq_radius = (radius && radius > 0.0) ? *radius * *radius : -1.0;

    const auto &node = _mesh.nodes[node_id];
    const auto &node_p = node.point;

    // Add the first edge.
    int e_id_1st = node.edges.front();
    const auto &e_1st = _mesh.edges[e_id_1st];
    int v_l_id_1st = e_1st.nodes[0];
    int v_r_id_1st = e_1st.nodes[1];
    if (_mesh.nodes[v_l_id_1st].edges.front() == e_id_1st) {
        std::swap(v_l_id_1st, v_r_id_1st);
    }
    if ((visible_nodes.empty() || visible_nodes.back() != v_r_id_1st) && (sq_radius < 0.0 || node_p.SquaredDistanceTo(_mesh.point(v_r_id_1st)) <= sq_radius)) {
        visible_nodes.push_back(v_r_id_1st);
    }
    if ((visible_nodes.empty() || visible_nodes.back() != v_l_id_1st) && (sq_radius < 0.0 || node_p.SquaredDistanceTo(_mesh.point(v_l_id_1st)) <= sq_radius)) {
        visible_nodes.push_back(v_l_id_1st);
    }

    for (int tri_id: node.triangles) {
        const auto &tri = _mesh.triangles[tri_id];
        for (int edge_id: tri.edges) {
            const auto &edge = _mesh.edges[edge_id];
            if (edge.nodes[0] != node_id && edge.nodes[1] != node_id) {
                int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

                // Establish the left and right restriction points.
                int rest_l_id = edge.nodes[0];
                int rest_r_id = edge.nodes[1];
                if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(rest_r_id), _mesh.point(rest_l_id))) {
                    std::swap(rest_l_id, rest_r_id);
                }

                // Expand the edge.
                ExpandEdgeVisibleNodes(1, node_p, rest_l_id, rest_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, sq_radius, visible_nodes, num_expansions);

                break;
            }
        }

    }

    // Add the last edge.
    int e_id_lst = node.edges.back();
    const auto &e_lst = _mesh.edges[e_id_lst];
    int v_l_id_lst = e_lst.nodes[0];
    int v_r_id_lst = e_lst.nodes[1];
    if (_mesh.nodes[v_l_id_lst].edges.front() == e_id_lst) {
        std::swap(v_l_id_lst, v_r_id_lst);
    }
    if (v_r_id_lst != node_id && (visible_nodes.empty() || visible_nodes.back() != v_r_id_lst)
        && (sq_radius < 0.0 || node_p.SquaredDistanceTo(_mesh.point(v_r_id_lst)) <= sq_radius)) {
        visible_nodes.push_back(v_r_id_lst);
    }
    if (v_l_id_lst != node_id && (visible_nodes.empty() || visible_nodes.back() != v_l_id_lst)
        && (sq_radius < 0.0 || node_p.SquaredDistanceTo(_mesh.point(v_l_id_lst)) <= sq_radius)) {
        visible_nodes.push_back(v_l_id_lst);
    }
}

void TriVis::ExpandEdgeVisibleNodes(
    int level,
    const FPoint &q,
    int rest_l_id,
    int rest_r_id,
    int curr_edge_id,
    int curr_edge_tri_id,
    double sq_radius,
    std::vector<int> &visible_nodes,
    int &num_expansions
) const {

    // current edge (struct)
    const auto &curr_edge = _mesh.edges[curr_edge_id];

    bool too_far_away = sq_radius >= 0.0 && PointSegmentSquaredDistance(q, _mesh.point(curr_edge.nodes[0]), _mesh.point(curr_edge.nodes[1])) > sq_radius;

    if (too_far_away || curr_edge.is_obstacle()) {
        // Report the edge and return.

        // Find out which one of the edge endpoints is 'left' and which one is 'right'.
        int v_l_id = curr_edge.nodes[0];
        int v_r_id = curr_edge.nodes[1];
        if (_mesh.nodes[v_l_id].edges.front() == curr_edge_id) {
            std::swap(v_l_id, v_r_id);
        }

        if (v_r_id == rest_r_id && (visible_nodes.empty() || visible_nodes.back() != v_r_id) && (sq_radius < 0.0 || q.SquaredDistanceTo(_mesh.point(v_r_id)) <= sq_radius)) {
            visible_nodes.push_back(v_r_id);
        }
        if (v_l_id == rest_l_id && (visible_nodes.empty() || visible_nodes.back() != v_l_id) && (sq_radius < 0.0 || q.SquaredDistanceTo(_mesh.point(v_l_id)) <= sq_radius)) {
            visible_nodes.push_back(v_l_id);
        }

        return;
    }

    // Record proper expansion (the expansion is not proper if the current edge is an obstacle or too far away).
    ++num_expansions;

    // Expand neighboring edges.

    // current left restriction point
    const auto &rest_l_p = _mesh.point(rest_l_id);
    // current right restriction point
    const auto &rest_r_p = _mesh.point(rest_r_id);

    // current triangle index
    int tri_id = curr_edge.triangles[curr_edge_tri_id];
    // current triangle (struct)
    const auto &tri = _mesh.triangles[tri_id];
    // node (its index) opposite to the current edge in current triangle
    int opp_id = curr_edge.opposites[curr_edge_tri_id];
    // point corresponding to the opposite node
    const auto &opp_p = _mesh.point(opp_id);
    // index (0, 1, or 2) of the current edge for member 'edges' of the current triangle
    int tri_edge_id = tri.edges[0] == curr_edge_id ? 0 : (tri.edges[1] == curr_edge_id ? 1 : 2);

    // index of the edge on right of the current edge
    int edge_r_id = tri.edges[(tri_edge_id + 1) % 3];
    // edge (struct) on right of the current edge
    const auto &edge_r = _mesh.edges[edge_r_id];
    // index of the other node (not opp) that makes edge_r
    int edge_r_not_opp_id = edge_r.nodes[edge_r.nodes[0] == opp_id ? 1 : 0];
    // true if the edge is obstacle
    bool edge_r_is_obstacle = edge_r.is_obstacle();

    // index of the edge on left of the current edge
    int edge_l_id = tri.edges[(tri_edge_id + 2) % 3];
    // edge (struct) on left of the current edge
    const auto &edge_l = _mesh.edges[edge_l_id];
    // index of the other node (not opp) that makes edge_l
    int edge_l_not_opp_id = edge_l.nodes[edge_l.nodes[0] == opp_id ? 1 : 0];
    // true if the edge is obstacle
    bool edge_l_is_obstacle = edge_l.is_obstacle();

    // true iff the opp_p satisfies restriction given by the left point
    Orientation o_q_rest_l_opp = Orient(q, rest_l_p, opp_p);
    // true iff the opp_p satisfies restriction given by the right point
    Orientation o_q_rest_r_opp = Orient(q, rest_r_p, opp_p);

    // Now decide if the other edges (left=l and right=r of the current edge) of the current triangle will be expanded.

    if (o_q_rest_r_opp != Orientation::kRightTurn) { // the right edge is at least partially visible
        int new_rest_l_id = rest_l_id;
        if (o_q_rest_l_opp != Orientation::kLeftTurn) {
            // update left restriction point if opp is more or the same way restricting
            new_rest_l_id = opp_id;
        }
        int new_edge_tri_id = edge_r.triangles[0] == tri_id ? 1 : 0;

        // *** note: right restriction node will always stay the same

        ExpandEdgeVisibleNodes(level + 1, q, new_rest_l_id, rest_r_id, edge_r_id, new_edge_tri_id, sq_radius, visible_nodes, num_expansions);
    }

    if (o_q_rest_l_opp != Orientation::kLeftTurn) { // the left edge is at least partially visible
        int new_rest_r_id = rest_r_id;
        if (o_q_rest_r_opp != Orientation::kRightTurn) {
            // update right restriction point if opp is more or the same way restricting
            new_rest_r_id = opp_id;
        }
        int new_edge_tri_id = edge_l.triangles[0] == tri_id ? 1 : 0;

        // *** note: left restriction node will always stay the same

        ExpandEdgeVisibleNodes(level + 1, q, rest_l_id, new_rest_r_id, edge_l_id, new_edge_tri_id, sq_radius, visible_nodes, num_expansions);
    }
}

bool TriVis::VisibilityBetween(
    const FPoint &q,
    const FPoint &t,
    int &num_expansions
) const {

    num_expansions = 0;

    // Find triangle where q is located.
    std::optional<int> tri_id_opt = _bucketing.FindTriangle(q, _triangles);
    if (!tri_id_opt) {
        // Return false in case the triangle could not be found.
        return false;
    }
    int tri_id = *tri_id_opt;

    // Use the node version in case q is a node of the triangle.
    const auto &tri = _mesh.triangles[tri_id];
    for (int node_id: tri.nodes) {
        if (q == _mesh.point(node_id)) {
            return VisibilityBetween(node_id, t, num_expansions);
        }
    }

    for (int edge_id: tri.edges) {
        const auto &edge = _mesh.edges[edge_id];
        int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

        // Establish the left and right restriction points.
        int node_l_id = edge.nodes[0];
        int node_r_id = edge.nodes[1];
        if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(node_r_id), _mesh.point(node_l_id))) {
            std::swap(node_l_id, node_r_id);
        }
        const auto &node_l_p = _mesh.point(node_l_id);
        const auto &node_r_p = _mesh.point(node_r_id);

        if (IsPointInCone(t, node_l_p, q, node_r_p)) {
            if (!TurnsLeft(node_l_p, node_r_p, t)) {
                // target lies in the same triangle as query
                return true;
            } else if (!edge.is_obstacle()) {
                return ExpandEdgeVisibilityBetween(1, q, t, node_l_id, node_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, num_expansions);
            }
        }
    }
    return false;
}

bool TriVis::VisibilityBetween(
    int node_id,
    const FPoint &tp,
    int &num_expansions
) const {

    num_expansions = 0;

    const auto &node = _mesh.nodes[node_id];
    const auto &node_p = node.point;

    // for each triangle 'tri' adjacent to the node ...
    for (auto tri_id: node.triangles) {

        // for each edge of triangle 'tri' ...
        for (auto edge_id: _mesh.triangles[tri_id].edges) {

            const auto &edge = _mesh.edges[edge_id];
            int edge_tri_id = edge.triangles[0] == tri_id ? 0 : 1;

            int node_l_id = edge.nodes[0];
            int node_r_id = edge.nodes[1];

            // assume edge is opposite to node
            if (node_l_id != node_id && node_r_id != node_id) {

                // Establish the left and right restriction points.
                if (!TurnsLeft(_mesh.point(edge.opposites[edge_tri_id]), _mesh.point(node_r_id), _mesh.point(node_l_id))) {
                    std::swap(node_l_id, node_r_id);
                }
                const auto &node_l_p = _mesh.point(node_l_id);
                const auto &node_r_p = _mesh.point(node_r_id);

                if (IsPointInCone(node_l_p, node_p, node_r_p, tp)) {
                    if (!TurnsLeft(node_l_p, node_r_p, tp)) {
                        // target lies in the same triangle as query
                        return true;
                    } else if (!edge.is_obstacle()) {
                        return ExpandEdgeVisibilityBetween(1, node_p, tp, node_l_id, node_r_id, edge_id, edge_tri_id == 0 ? 1 : 0, num_expansions);
                    }
                }
            }
        }
    }

    return false;
}

void TriVis::AppendNotCollinearWithIntersection(
    const FPoint &q,
    const AbstractVisibilityRegionVertex &v,
    int edge_flag,
    bool is_last,
    bool fast_mode,
    RadialVisibilityRegion &visibility_region
) const {
    int vertex_flag;
    FPoint p, aux;
    if (v.is_intersection) {
        // Compute the intersection.
        const auto &b = _mesh.point(v.id_b);
        const auto &c = _mesh.point(v.id_c);
        const auto &d = _mesh.point(v.id_d);
        bool intersection_not_found = false;
        if (fast_mode) {
            // LineLineIntersectionNotCollinear is faster (but less safe) than RaySegmentIntersection
            intersection_not_found = LineLineIntersectionNotCollinear(q, b, c, d, p);
        }
        if (!fast_mode || intersection_not_found) {
            char code = RaySegmentIntersection(q, b, c, d, p, aux);
            if (code == '0' || code == 'c' || code == 'e') {
                std::cerr.precision(std::numeric_limits<double>::max_digits10);
                if (code == 'e') {
                    std::cerr << "[AppendNotCollinearWithIntersection] RaySeg intersection NOT found although it should REALLY exist (Code: " << code << "): ";
                } else {
                    std::cerr << "[AppendNotCollinearWithIntersection] RaySeg intersection NOT found although it should PROBABLY exist (Code: " << code << "): ";
                }
                std::cerr << std::fixed << q << ", " << b << ", " << c << ", " << d << ".\n";
                return;
            }
        }
        vertex_flag = -1;
    } else {
        p = _mesh.point(v.id);
        vertex_flag = v.id;
    }

    // If the points are exactly equal, ignore the new point.
    if (!visibility_region.vertices.empty()) {
        if (p == visibility_region.vertices.back().point) {
            return;
        }
        if (is_last && p == visibility_region.vertices.front().point) {
            visibility_region.vertices.front().edge_flag = edge_flag;
            return;
        }
    }
    // Not collinear append.
    if (visibility_region.vertices.size() > 1) {
        const auto &p_prev_prev = (visibility_region.vertices.crbegin() + 1)->point;
        const auto &p_prev = visibility_region.vertices.back().point;
        if (edge_flag == visibility_region.vertices.back().edge_flag && Collinear(p_prev_prev, p_prev, p) && Extends(p_prev_prev, p_prev, p)) {
            visibility_region.vertices.back().vertex_flag = vertex_flag;
            visibility_region.vertices.back().point = p;
            return;
        }
    }
    visibility_region.vertices.push_back({vertex_flag, edge_flag, p});
}

bool TriVis::ExpandEdgeVisibilityBetween(
    int level,
    const FPoint &q,
    const FPoint &t,
    int node_l_id,
    int node_r_id,
    int curr_edge_id,
    int curr_edge_tri_id,
    int &num_expansions
) const {

    ++num_expansions;

    const auto &edge = _mesh.edges[curr_edge_id];
    const auto &node_l_p = _mesh.point(node_l_id);
    const auto &node_r_p = _mesh.point(node_r_id);
    int tri_id = edge.triangles[curr_edge_tri_id];
    const auto &tri = _mesh.triangles[tri_id];
    int opp_id = edge.opposites[curr_edge_tri_id];
    const auto &opp = _mesh.point(opp_id);
    int tri_edge_id = tri.edges[0] == curr_edge_id ? 0 : (tri.edges[1] == curr_edge_id ? 1 : 2);
    // *** note: tri edges are in counter-clockwise order
    int edge_r_id = tri.edges[(tri_edge_id + 1) % 3];
    const auto &edge_r = _mesh.edges[edge_r_id];
    int edge_l_id = tri.edges[(tri_edge_id + 2) % 3];
    const auto &edge_l = _mesh.edges[edge_l_id];
    if (IsPointInCone(t, node_l_p, q, opp) && TurnsLeft(q, opp, node_l_p)) {
        // tp is in the left cone.
        if (!TurnsLeft(node_l_p, opp, t)) {
            // tp is in the current triangle.
            return true;
        } else if (!edge_l.is_obstacle()) {
            // expand left edge
            return ExpandEdgeVisibilityBetween(level + 1, q, t, node_l_id, opp_id, edge_l_id, edge_l.triangles[0] == tri_id ? 1 : 0, num_expansions);
        } else if (!edge_r.is_obstacle() && IsPointInCone(t, opp, q, node_r_p) && TurnsRight(q, opp, node_r_p)) {
            // if the left is obstacle but the right is not and p1 is in both cones, then expand right edge
            return ExpandEdgeVisibilityBetween(level + 1, q, t, opp_id, node_r_id, edge_r_id, edge_r.triangles[0] == tri_id ? 1 : 0, num_expansions);
        } else {
            return false;
        }
    } else { // tp must be in the right cone.
        if (!TurnsRight(node_r_p, opp, t)) {
            // tp is in the current triangle.
            return true;
        } else if (!edge_r.is_obstacle()) {
            // expand right edge
            return ExpandEdgeVisibilityBetween(level + 1, q, t, opp_id, node_r_id, edge_r_id, edge_r.triangles[0] == tri_id ? 1 : 0, num_expansions);
        } else {
            return false;
        }
    }
}

void TriVis::ClearStructures() {
    _mesh.nodes.clear();
    _mesh.edges.clear();
    _mesh.triangles.clear();
    _triangles.clear();
}
