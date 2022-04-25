/**
 * File:   trivis.h
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_TRI_VIS_H_
#define TRIVIS_CORE_TRI_VIS_H_

#include "trivis/core/geom/poly_map.h"
#include "trivis/core/geom/visibility_region.h"

#include "trivis/core/triangulation/tri_mesh.h"

#include "trivis/core/bucketing/bucketing.h"

#include <memory>
#include <utility>

namespace trivis::core {

class TriVis {
public:

    TriVis() = default;

    /**
     * Copy constructor.
     */
    TriVis(const TriVis &) = default;

    /**
     * Move constructor.
     */
    TriVis(TriVis &&) noexcept = default;

    /**
     * Copy operator.
     */
    TriVis &operator=(const TriVis &) = default;

    /**
     * Move operator.
     */
    TriVis &operator=(TriVis &&) noexcept = default;

    /**
     * Destructor.
     */
    virtual ~TriVis() = default;

    [[nodiscard]] const auto &map() const { return _map; }

    [[nodiscard]] const auto &mesh() const { return _mesh; }

    [[nodiscard]] const auto &triangles() const { return _triangles; }

    [[nodiscard]] const auto &bucketing() const { return _bucketing; }

    void SetMap(geom::PolyMap map);

    void TriangulateMapConstrainedDelaunay();

    void FillBucketing();

    void OptimizeBuckets();

    [[nodiscard]] geom::RadialVisibilityRegion ConvertToVisibilityRegion(
        const geom::AbstractVisibilityRegion &abstract,
        bool fast_mode = false
    ) const;

    [[nodiscard]] static geom::RadialVisibilityRegion ConvertToRadialVisibilityRegion(
        double radius,
        const geom::RadialVisibilityRegion &visibility_region
    );

    [[nodiscard]] static geom::FPolygon ConvertToPolygon(
        const geom::RadialVisibilityRegion &visibility_region
    );

    bool ComputeVisibilityRegion(
        const geom::FPoint &q,
        std::optional<double> radius,
        geom::AbstractVisibilityRegion &visibility_region,
        int &num_expansions
    ) const;

    void ComputeVisibilityRegion(
        int node_id,
        std::optional<double> radius,
        geom::AbstractVisibilityRegion &visibility_region,
        int &num_expansions
    ) const;

    bool ComputeVisibleNodes(
        const geom::FPoint &q,
        std::optional<double> radius,
        std::vector<int> &visible_nodes,
        int &num_expansions
    ) const;

    void ComputeVisibleNodes(
        int node_id,
        std::optional<double> radius,
        std::vector<int> &visible_nodes,
        int &num_expansions
    ) const;

    [[nodiscard]] bool VisibilityBetween(
        const geom::FPoint &q,
        const geom::FPoint &t,
        int &num_expansions
    ) const;

    [[nodiscard]] bool VisibilityBetween(
        int node_id,
        const geom::FPoint &tp,
        int &num_expansions
    ) const;

private:

    geom::PolyMap _map;
    triangulation::TriMesh _mesh;
    geom::FPolygons _triangles;
    bucketing::Bucketing _bucketing;

    void AppendNotCollinearWithIntersection(
        const geom::FPoint &q,
        const geom::AbstractVisibilityRegionVertex &v,
        int edge_flag,
        bool is_last,
        bool fast_mode,
        geom::RadialVisibilityRegion &visibility_region
    ) const;

    void ExpandEdgeVisibilityRegion(
        int level,
        const geom::FPoint &q,
        int rest_l_id,
        int rest_r_id,
        int curr_edge_id,
        int curr_edge_tri_id,
        double sq_radius,
        geom::AbstractVisibilityRegion &visibility_region,
        int &num_expansions
    ) const;

    void ExpandEdgeVisibleNodes(
        int level,
        const geom::FPoint &q,
        int rest_l_id,
        int rest_r_id,
        int curr_edge_id,
        int curr_edge_tri_id,
        double sq_radius,
        std::vector<int> &visible_nodes,
        int &num_expansions
    ) const;

    [[nodiscard]] bool ExpandEdgeVisibilityBetween(
        int level,
        const geom::FPoint &q,
        const geom::FPoint &t,
        int node_l_id,
        int node_r_id,
        int curr_edge_id,
        int curr_edge_tri_id,
        int &num_expansions
    ) const;

    void ClearStructures();

};

}

#endif //TRIVIS_CORE_TRI_VIS_H_
