/**
 * File:   tri_mesh.cc
 *
 * Date:   24.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/triangulation/tri_mesh.h"

#include <cassert>
#include <iostream>

#include "trivis/core/geom/robust_geometry.h"

using namespace trivis::core;
using namespace trivis::core::geom;
using namespace trivis::core::triangulation;

void triangulation::SortStructures(TriMesh &mesh) {
    auto &edges = mesh.edges;
    auto &nodes = mesh.nodes;
    auto &triangles = mesh.triangles;
    auto TurnsRight = [mesh](int i, int j, int k) -> bool {
        return geom::TurnsRight(mesh.point(i), mesh.point(j), mesh.point(k));
    };

    // sort edges in each triangle counterclockwise
    for (auto &tri: triangles) {
        int a = tri.edges[0];
        int b = tri.edges[1];
        int a1 = edges[a].nodes[0];
        int a2 = edges[a].nodes[1];
        int b1 = edges[b].nodes[0];
        int b2 = edges[b].nodes[1];
        if ((a1 == b1 && TurnsRight(a2, a1, b2)) ||
            (a1 == b2 && TurnsRight(a2, a1, b1)) ||
            (a2 == b1 && TurnsRight(a1, a2, b2)) ||
            (a2 == b2 && TurnsRight(a1, a2, b1))) {
            tri.edges[0] = b;
            tri.edges[1] = a;
        }
    }

    // sort edges and triangles in each node counterclockwise
    for (int node_idx = 0; node_idx < nodes.size(); ++node_idx) {
        auto &node = nodes[node_idx];
        const auto &node_p = node.point;
        assert(!node.edges.empty());
        std::cerr.precision(std::numeric_limits<double>::max_digits10);
        std::vector<int> new_edges;
        std::vector<int> new_triangles;
        int curr_e_idx = -1;
        for (int e_idx: node.edges) {
            const auto &edge = edges[e_idx];
            assert(edge.nodes[0] == node_idx || edge.nodes[1] == node_idx);
            if (edge.is_obstacle()) {
                const auto &e_other_node_p = nodes[edge.nodes[0] == node_idx ? edge.nodes[1] : edge.nodes[0]].point;
                const auto &e_opp_p = nodes[edge.opposites[0]].point;
                if (!geom::TurnsRight(node_p, e_other_node_p, e_opp_p)) {
                    curr_e_idx = e_idx;
                    break;
                }
            }
        }
        assert(curr_e_idx != -1);
        int curr_tri_idx = edges[curr_e_idx].triangles[0];
        while (true) {
            new_edges.push_back(curr_e_idx);
            new_triangles.push_back(curr_tri_idx);
            // update current edge
            const auto &curr_tri = triangles[curr_tri_idx];
            int curr_tri_curr_e_idx = curr_tri.edges[0] == curr_e_idx ? 0 : (curr_tri.edges[1] == curr_e_idx ? 1 : 2);
            curr_e_idx = curr_tri.edges[(curr_tri_curr_e_idx + 2) % 3];
            if (!edges[curr_e_idx].is_obstacle()) {
                // update current triangle
                curr_tri_idx = edges[curr_e_idx].triangles[0] == curr_tri_idx ? edges[curr_e_idx].triangles[1] : edges[curr_e_idx].triangles[0];
            } else {
                // reached the last edge
                new_edges.push_back(curr_e_idx);
                break;
            }
        }
        assert(new_edges.size() == node.edges.size());
        assert(new_triangles.size() == node.triangles.size());
        node.edges = std::move(new_edges);
        node.triangles = std::move(new_triangles);
    }
}
