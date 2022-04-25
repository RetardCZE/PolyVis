/**
 * File:   bucketing.h
 *
 * Date:   24.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_BUCKETING_BUCKETING_H_
#define TRIVIS_CORE_BUCKETING_BUCKETING_H_

#include <array>
#include <optional>

#include "trivis/core/bucketing/grid.h"

#include "trivis/core/geom/geom_types.h"

namespace trivis::core::bucketing {

void FillBuckets(
    Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale,
    const geom::FPolygons &triangles
);

void OrderBucketTrianglesByIntersectionArea(
    Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale,
    const geom::FPolygons &triangles
);

void RemoveDuplicateBucketTriangles(
    Grid &buckets
);

std::optional<int> FindTriangle(
    const geom::FPoint &q,
    const geom::FPolygons &triangles,
    const Grid &buckets,
    const geom::FLimits &lim,
    double x_scale,
    double y_scale
);

class Bucketing {
public:

    static constexpr double kDefaultBucketSize = 0.75;

    void Init(geom::FLimits lim, double bucket_size = kDefaultBucketSize) {
        _lim = lim;
        _ncol = static_cast<int>(std::ceil((lim.x_max - lim.x_min) / bucket_size));
        _nrow = static_cast<int>(std::ceil((lim.y_max - lim.y_min) / bucket_size));
        _x_scale = (lim.x_max - lim.x_min) / static_cast<double>(_ncol);
        _y_scale = (lim.y_max - lim.y_min) / static_cast<double>(_nrow);
        _buckets.Resize(_nrow, _ncol);
    }

    void FillBuckets(const geom::FPolygons &triangles) {
        bucketing::FillBuckets(_buckets, _lim, _x_scale, _y_scale, triangles);
    }

    void OrderBucketTrianglesByIntersectionArea(const geom::FPolygons &triangles) {
        bucketing::OrderBucketTrianglesByIntersectionArea(_buckets, _lim, _x_scale, _y_scale, triangles);
    }

    void RemoveDuplicateBucketTriangles() {
        bucketing::RemoveDuplicateBucketTriangles(_buckets);
    }

    [[nodiscard]] std::optional<int> FindTriangle(const geom::FPoint &q, const geom::FPolygons &triangles) const {
        return bucketing::FindTriangle(q, triangles, _buckets, _lim, _x_scale, _y_scale);
    }

private:
    geom::FLimits _lim;
    int _nrow = 0;
    int _ncol = 0;
    double _x_scale = 0.0;
    double _y_scale = 0.0;
    Grid _buckets;
};

}

#endif //TRIVIS_CORE_BUCKETING_BUCKETING_H_
