/**
 * File:   bucketing.cc
 *
 * Date:   24.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/bucketing/bucketing.h"

#include <iostream>

#include <set>
#include <array>
#include <algorithm>

#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/geom/intersections.h"

#include "trivis/core/utils/clipper_utils.h"
#include "trivis/core/utils/generic_utils.h"

using namespace trivis::core;
using namespace trivis::core::geom;
using namespace trivis::core::bucketing;

inline int ToIdxX(
    double x,
    double x_min,
    double x_scale,
    int ncol
) {
    int result = std::floor((x - x_min) / x_scale);
    if (result < 0) return 0;
    if (result >= ncol) return ncol - 1;
    return result;
}

inline int ToIdxY(
    double y,
    double y_min,
    double y_scale,
    int nrow
) {
    int result = std::floor((y - y_min) / y_scale);
    if (result < 0) return 0;
    if (result >= nrow) return nrow - 1;
    return result;
}

void FillLine(Grid &buckets, int id, int x_start, int x_end, int y) {
    for (int x = x_start; x <= x_end; ++x) {
        buckets(x, y).push_back(id);
    }
}

void FillFlatBottomTriangle(
    Grid &buckets,
    double x_min,
    double x_scale,
    const std::vector<std::array<geom::FPoint, 2>> &h_buck_lines,
    const geom::FPoint &v1,
    const geom::FPoint &v2,
    const geom::FPoint &v3,
    int x1,
    int x2,
    int x3,
    int y1,
    int y3,
    int id
) {
    std::vector<std::pair<int, int>> x_candid;
    x_candid.emplace_back(x1, x2);
    for (int i = y1 + 1; i <= y3; ++i) {
        const auto &h_line = h_buck_lines[i];
        geom::FPoint q1, q2, aux;
        char c1 = geom::SegmentSegmentIntersection(v1, v3, h_line[0], h_line[1], q1, aux);
        if (c1 == '0' || c1 == 'c' || c1 == 'e') {
            std::cerr.precision(std::numeric_limits<double>::max_digits10);
            std::cerr << std::fixed << "[FillFlatBottomTriangle] SegSeg intersection NOT found (Code: " << c1 << "): ";
            std::cerr << v1 << ", " << v3 << ", " << h_line[0] << ", " << h_line[1] << ".\n";
        }
        char c2 = geom::SegmentSegmentIntersection(v2, v3, h_line[0], h_line[1], q2, aux);
        if (c2 == '0' || c2 == 'c' || c2 == 'e') {
            std::cerr.precision(std::numeric_limits<double>::max_digits10);
            std::cerr << std::fixed << "[FillFlatBottomTriangle] SegSeg intersection NOT found (Code: " << c2 << "): ";
            std::cerr << v2 << ", " << v3 << ", " << h_line[0] << ", " << h_line[1] << ".\n";
        }
        x_candid.emplace_back(ToIdxX(q1.x, x_min, x_scale, buckets.ncol()), ToIdxX(q2.x, x_min, x_scale, buckets.ncol()));
    }
    x_candid.emplace_back(x3, x3);
    for (int yy = y1, q_idx = 0; yy <= y3; ++yy, ++q_idx) {
        int xx1 = x_candid[q_idx].first;
        int xx2 = x_candid[q_idx].second;
        int xx3 = x_candid[q_idx + 1].first;
        int xx4 = x_candid[q_idx + 1].second;
        FillLine(buckets, id, std::min({xx1, xx2, xx3, xx4}), std::max({xx1, xx2, xx3, xx4}), yy);
    }
}

void FillFlatTopTriangle(
    Grid &buckets,
    double x_min,
    double x_scale,
    const std::vector<std::array<geom::FPoint, 2>> &h_buck_lines,
    const geom::FPoint &v1,
    const geom::FPoint &v2,
    const geom::FPoint &v3,
    int x1,
    int x2,
    int x3,
    int y1,
    int y3,
    int id
) {
    std::vector<std::pair<int, int>> x_candid;
    x_candid.emplace_back(x1, x1);
    for (int i = y1 + 1; i <= y3; ++i) {
        const auto &h_line = h_buck_lines[i];
        geom::FPoint q2, q3, aux;
        char c1 = geom::SegmentSegmentIntersection(v1, v2, h_line[0], h_line[1], q2, aux);
        if (c1 == '0' || c1 == 'c' || c1 == 'e') {
            std::cerr.precision(std::numeric_limits<double>::max_digits10);
            std::cerr << std::fixed << "[FillFlatTopTriangle] SegSeg intersection NOT found (Code: " << c1 << "): ";
            std::cerr << v1 << ", " << v2 << ", " << h_line[0] << ", " << h_line[1] << ".\n";
        }
        char c2 = geom::SegmentSegmentIntersection(v1, v3, h_line[0], h_line[1], q3, aux);
        if (c2 == '0' || c2 == 'c' || c2 == 'e') {
            std::cerr.precision(std::numeric_limits<double>::max_digits10);
            std::cerr << std::fixed << "[FillFlatTopTriangle] SegSeg intersection NOT found (Code: " << c2 << "): ";
            std::cerr << v1 << ", " << v3 << ", " << h_line[0] << ", " << h_line[1] << ".\n";
        }
        x_candid.emplace_back(ToIdxX(q2.x, x_min, x_scale, buckets.ncol()), ToIdxX(q3.x, x_min, x_scale, buckets.ncol()));
    }
    x_candid.emplace_back(x2, x3);
    for (int yy = y1, q_idx = 0; yy <= y3; ++yy, ++q_idx) {
        int xx1 = x_candid[q_idx].first;
        int xx2 = x_candid[q_idx].second;
        int xx3 = x_candid[q_idx + 1].first;
        int xx4 = x_candid[q_idx + 1].second;
        FillLine(buckets, id, std::min({xx1, xx2, xx3, xx4}), std::max({xx1, xx2, xx3, xx4}), yy);
    }
}

/**
 * Inspired by
 * http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html
 */
void FillTriangle(
    Grid &buckets,
    double x_min,
    double y_min,
    double x_scale,
    double y_scale,
    const std::vector<std::array<geom::FPoint, 2>> &h_buck_lines,
    geom::FPolygon triangle,
    int id
) {
    std::sort(triangle.begin(), triangle.end(), [](const geom::FPoint &a, const geom::FPoint &b) { return a.y < b.y; });
    const auto &v1 = triangle[0], &v2 = triangle[1], &v3 = triangle[2];
    const int x1 = ToIdxX(v1.x, x_min, x_scale, buckets.ncol()), x2 = ToIdxX(v2.x, x_min, x_scale, buckets.ncol()), x3 = ToIdxX(v3.x, x_min, x_scale, buckets.ncol());
    const int y1 = ToIdxY(v1.y, y_min, y_scale, buckets.nrow()), y2 = ToIdxY(v2.y, y_min, y_scale, buckets.nrow()), y3 = ToIdxY(v3.y, y_min, y_scale, buckets.nrow());
    if (y1 == y3) { // Check for trivial case of one-line triangle.
        FillLine(buckets, id, std::min({x1, x2, x3}), std::max({x1, x2, x3}), y2);
    } else if (y1 == y2) { // Check for trivial case of bottom-flat triangle.
        FillFlatBottomTriangle(buckets, x_min, x_scale, h_buck_lines, v1, v2, v3, x1, x2, x3, y1, y3, id);
    } else if (y2 == y3) { // Check for trivial case of top-flat triangle.
        FillFlatTopTriangle(buckets, x_min, x_scale, h_buck_lines, v1, v2, v3, x1, x2, x3, y1, y3, id);
    } else { // General case - split the triangle into two: top-flat and bottom-flat.
        double den = v3.y - v1.y;
        if (den == 0.0) {
            std::cerr.precision(std::numeric_limits<double>::max_digits10);
            std::cerr << std::fixed << "[FillTriangle] Could not split triangle! Triangle " << v1 << ", " << v2 << ", " << v3 << " is probably invalid.\n";
        }
        const auto v4 = geom::MakePoint(v1.x + ((v2.y - v1.y) / den) * (v3.x - v1.x), v2.y);
        const int x4 = ToIdxX(v4.x, x_min, x_scale, buckets.ncol());
        const int y4 = ToIdxY(v4.y, y_min, y_scale, buckets.nrow());
        FillFlatBottomTriangle(buckets, x_min, x_scale, h_buck_lines, v4, v2, v3, x4, x2, x3, y4, y3, id);
        FillFlatTopTriangle(buckets, x_min, x_scale, h_buck_lines, v1, v2, v4, x1, x2, x4, y1, y4, id);
    }
}

void bucketing::FillBuckets(
    Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale,
    const geom::FPolygons &triangles
) {
    // Prepare structures.
    std::vector<std::array<geom::FPoint, 2>> h_buck_lines;
    h_buck_lines.reserve(buckets.nrow());
    for (int i = 0; i < buckets.nrow(); ++i) {
        double y = lim.y_min + i * y_scale;
        h_buck_lines.push_back({geom::MakePoint(lim.x_min, y), geom::MakePoint(lim.x_max, y)});
    }
    // Fill buckets.
    int idx = 0;
    for (const auto &triangle: triangles) {
        FillTriangle(buckets, lim.x_min, lim.y_min, x_scale, y_scale, h_buck_lines, triangle, idx++);
    }
}

void bucketing::RemoveDuplicateBucketTriangles(
    Grid &buckets
) {
    for (int i = 0; i < buckets.ncol(); ++i) {
        for (int j = 0; j < buckets.nrow(); ++j) {
            std::vector<int> &bucket = buckets(i, j);
            const auto bucket_set = std::set<int>(bucket.begin(), bucket.end());
            bucket = std::vector<int>(bucket_set.begin(), bucket_set.end());
        }
    }
}

void bucketing::OrderBucketTrianglesByIntersectionArea(
    Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale,
    const geom::FPolygons &triangles
) {
    ClipperLib::Clipper clipper;
    double normalizer = std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    double x = lim.x_min;
    for (int i = 0; i < buckets.ncol(); ++i) {
        double y = lim.y_min;
        for (int j = 0; j < buckets.nrow(); ++j) {
            auto &bucket_tri_ids = buckets(i, j);
            geom::FPolygon bucket_rect = {{x, y}, {x + x_scale, y}, {x + x_scale, y + y_scale}, {x, y + y_scale}};
            auto bucket_rect_c = utils::Geom2Clipper(bucket_rect, normalizer, {lim.x_min, lim.y_min});
            std::vector<double> intersect_areas(bucket_tri_ids.size(), 0.0);
            for (int k = 0; k < bucket_tri_ids.size(); ++k) {
                auto triangle_c = utils::Geom2Clipper(triangles[bucket_tri_ids[k]], normalizer, {lim.x_min, lim.y_min});
                ClipperLib::Paths intersection;
                clipper.AddPath(bucket_rect_c, ClipperLib::ptSubject, true);
                clipper.AddPath(triangle_c, ClipperLib::ptClip, true);
                clipper.Execute(ClipperLib::ctIntersection, intersection, ClipperLib::pftNonZero);
                clipper.Clear();
                if (!intersection.empty()) {
                    intersect_areas[k] = ClipperLib::Area(intersection[0]); // does not need to be in scale with the original representation
                }
            }
            bucket_tri_ids = utils::SortBy(bucket_tri_ids, intersect_areas);
            std::reverse(bucket_tri_ids.begin(), bucket_tri_ids.end());
            y += y_scale;
        }
        x += x_scale;
    }
}

std::optional<int> bucketing::FindTriangle(
    const geom::FPoint &q,
    const geom::FPolygons &triangles,
    const Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale
) {
    int x = ToIdxX(q.x, lim.x_min, x_scale, buckets.ncol());
    int y = ToIdxY(q.y, lim.y_min, y_scale, buckets.nrow());
    if (!(0 <= x && x < buckets.ncol() && 0 <= y && y < buckets.nrow())) {
        return std::nullopt;
    }
    // Try exact.
    for (int id: buckets(x, y)) {
        if (geom::PointTriangleRelation(q, triangles[id][0], triangles[id][1], triangles[id][2]) != '0') {
            return id;
        }
    }
    // Try with eps.
    for (const double &eps : {1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3}) {
        for (int id: buckets(x, y)) {
            if (geom::PointTriangleRelationWithEps(q, triangles[id][0], triangles[id][1], triangles[id][2], eps) != '0') {
                return id;
            }
        }
    }
    return std::nullopt;
}
