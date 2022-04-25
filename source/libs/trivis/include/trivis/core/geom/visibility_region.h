/**
 * File:   visibility_region.h
 *
 * Date:   30.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_VISIBILITY_REGION_H_
#define TRIVIS_CORE_VISIBILITY_REGION_H_

#include <vector>

#include "trivis/core/geom/geom_types.h"

namespace trivis::core::geom {

struct AbstractVisibilityRegionVertex {
    bool is_intersection = false;
    int id = -1;
    int id_b = -1;
    int id_c = -1;
    int id_d = -1;
};

struct AbstractVisibilityRegionSegment {
    int id = -1;
    AbstractVisibilityRegionVertex v1;
    AbstractVisibilityRegionVertex v2;
};

struct AbstractVisibilityRegion {
    int seed_id = -1;
    geom::FPoint seed;
    std::vector<AbstractVisibilityRegionSegment> segments;
};

struct VisibilityRegionVertex {
    int vertex_flag;
    int edge_flag;
    geom::FPoint point;
};

struct RadialVisibilityRegion {
    double radius = -1;
    int seed_id = -1;
    geom::FPoint seed;
    std::vector<VisibilityRegionVertex> vertices;
};

void RemoveShortEdges(
    double min_edge_length,
    RadialVisibilityRegion &res
);

void RemoveAntennas(
    RadialVisibilityRegion &res
);

}

#endif //TRIVIS_CORE_VISIBILITY_REGION_H_
