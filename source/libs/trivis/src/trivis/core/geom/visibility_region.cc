/**
 * File:   visibility_region.cc
 *
 * Date:   13.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/geom/visibility_region.h"

#include <iostream>
#include <algorithm>

#include "trivis/core/geom/robust_geometry.h"

using namespace trivis::core;
using namespace trivis::core::geom;

void geom::RemoveShortEdges(double min_edge_length, RadialVisibilityRegion &res) {
    std::vector<VisibilityRegionVertex> new_vertices;
    for (int i_prev = static_cast<int>(res.vertices.size()) - 1, i = 0; i < res.vertices.size(); i_prev = i++) {
        const auto &v_prev = res.vertices[i_prev];
        const auto &v = res.vertices[i];
        if (v_prev.point.DistanceTo(v.point) < min_edge_length) {
            if (v.vertex_flag >= 0) {
                if (!new_vertices.empty()) {
                    new_vertices.pop_back();
                }
                new_vertices.push_back({v.vertex_flag, v_prev.edge_flag, v.point});
            }
        } else {
            new_vertices.push_back(v);
        }
    }
    res.vertices = std::move(new_vertices);
}

void geom::RemoveAntennas(
    RadialVisibilityRegion &res
) {
    int n_minus_1 = static_cast<int>(res.vertices.size()) - 1;
    std::vector<int> antenna_peaks;
    for (int i_prev = n_minus_1, i = 0; i < res.vertices.size(); i_prev = i++) {
        int i_next = i == n_minus_1 ? 0 : i + 1;
        const auto &v_prev = res.vertices[i_prev];
        const auto &v = res.vertices[i];
        const auto &v_next = res.vertices[i_next];
        if (Collinear(v_prev.point, v.point, v_next.point) && !Extends(v_prev.point, v.point, v_next.point)) {
            antenna_peaks.push_back(i);
        }
    }
    if (antenna_peaks.empty()) {
        return;
    }
    int n_antennas = static_cast<int>(antenna_peaks.size());
    std::vector<int> antenna_1st(n_antennas);
    std::vector<bool> antenna_1st_shifted(n_antennas, false);
    std::vector<int> antenna_end(n_antennas);
    std::vector<bool> antenna_end_shifted(n_antennas, false);
    for (int i = 0; i < n_antennas; ++i) {
        int antenna_peak = antenna_peaks[i];
        int antenna_1st_candid, antenna_end_candid;
        {   // Compute antenna_1st_candid.
            int j_curr = antenna_peak;
            while (true) {
                int j_prev = j_curr == 0 ? n_minus_1 : j_curr - 1;
                int j_prev_prev = j_prev == 0 ? n_minus_1 : j_prev - 1;
                const auto &v_curr = res.vertices[j_curr];
                const auto &v_prev = res.vertices[j_prev];
                const auto &v_prev_prev = res.vertices[j_prev_prev];
                if (!Collinear(v_curr.point, v_prev.point, v_prev_prev.point)) {
                    antenna_1st_candid = j_prev;
                    break;
                }
                j_curr = j_prev;
            }
        }
        {   // Compute antenna_end_candid.
            int j_curr = antenna_peak;
            while (true) {
                int j_next = j_curr == n_minus_1 ? 0 : j_curr + 1;
                int j_next_next = j_next == n_minus_1 ? 0 : j_next + 1;
                const auto &v_curr = res.vertices[j_curr];
                const auto &v_next = res.vertices[j_next];
                const auto &v_next_next = res.vertices[j_next_next];
                if (!Collinear(v_curr.point, v_next.point, v_next_next.point)) {
                    antenna_end_candid = j_next;
                    break;
                }
                j_curr = j_next;
            }
        }
        {   // Get antenna real 1st.
            int j_curr = antenna_1st_candid;
            while (j_curr < antenna_peak) {
                int j_next = j_curr == n_minus_1 ? 0 : j_curr + 1;
                const auto &v_curr = res.vertices[j_curr];
                const auto &v_next = res.vertices[j_next];
                if (!Extends(v_curr.point, v_next.point, res.vertices[antenna_end_candid].point)) {
                    antenna_1st[i] = j_curr;
                    break;
                }
                j_curr = j_next;
            }

        }
        antenna_1st_shifted[i] = (antenna_1st_candid != antenna_1st[i]);
        {   // Get antenna real end.
            int j_curr = antenna_end_candid;
            while (j_curr > antenna_peak) {
                int j_prev = j_curr == 0 ? n_minus_1 : j_curr - 1;
                const auto &v_curr = res.vertices[j_curr];
                const auto &v_prev = res.vertices[j_prev];
                if (!Extends(v_curr.point, v_prev.point, res.vertices[antenna_1st_candid].point)) {
                    antenna_end[i] = j_curr;
                    break;
                }
                j_curr = j_prev;
            }

        }
        antenna_end_shifted[i] = (antenna_end_candid != antenna_end[i]);
    }

    int antenna_cnt = 0;
    bool is_antenna = false;
    std::vector<VisibilityRegionVertex> new_vertices;
    for (int i = 0; i < res.vertices.size(); ++i) {
        const auto &v = res.vertices[i];
        if (is_antenna) {
            if (i == antenna_end[antenna_cnt]) {
                new_vertices.push_back(v);
                if (antenna_1st_shifted[antenna_cnt]) {
                    new_vertices.back().edge_flag = res.vertices[antenna_1st[antenna_cnt] == n_minus_1 ? 0 : antenna_1st[antenna_cnt] + 1].edge_flag;
                }
                ++antenna_cnt;
                is_antenna = false;
                continue;
            }
            if (i == n_minus_1) {
                std::reverse(new_vertices.begin(), new_vertices.end());
                for (int j = 0; j < antenna_end[antenna_cnt]; ++j) {
                    new_vertices.pop_back();
                }
                if (antenna_1st_shifted[antenna_cnt]) {
                    new_vertices.back().edge_flag = res.vertices[antenna_1st[antenna_cnt] == n_minus_1 ? 0 : antenna_1st[antenna_cnt] + 1].edge_flag;
                }
                std::reverse(new_vertices.begin(), new_vertices.end());
                break;
            }
            continue;
        }
        if (i == antenna_1st[antenna_cnt]) {
            new_vertices.push_back(v);
            is_antenna = true;
            continue;
        }
        new_vertices.push_back(v);
    }
    res.vertices = std::move(new_vertices);
}
