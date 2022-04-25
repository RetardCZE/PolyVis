/**
 * File:   poly_map.h
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_GEOM_POLY_MAP_H_
#define TRIVIS_CORE_GEOM_POLY_MAP_H_

#include <utility>

#include "trivis/core/geom/geom_types.h"
#include "trivis/core/geom/generic_geom_utils.h"

namespace trivis::core::geom {

class PolyMap {
public:

    PolyMap(FPolygon border, FPolygons holes) : _border(std::move(border)), _holes(std::move(holes)) {
        ComputeLimits(_border, _limits.x_min, _limits.x_max, _limits.y_min, _limits.y_max);
    }

    explicit PolyMap(FPolygon border) : PolyMap(std::move(border), FPolygons{}) {}

    /**
     * Default constructor.
     */
    PolyMap() = default;

    /**
     * Copy constructor.
     */
    PolyMap(const PolyMap &) = default;

    /**
     * Move constructor.
     */
    PolyMap(PolyMap &&) noexcept = default;

    /**
     * Copy operator.
     */
    PolyMap &operator=(const PolyMap &) = default;

    /**
     * Move operator.
     */
    PolyMap &operator=(PolyMap &&) noexcept = default;

    /**
     * Destructor.
     */
    virtual ~PolyMap() = default;

    [[nodiscard]] const auto &limits() const { return _limits; }

    [[nodiscard]] const auto &border() const { return _border; }

    [[nodiscard]] const auto &holes() const { return _holes; }

    void RemoveDuplicatePoints();

    void RemoveCollinearPoints();

    void ShiftToOrigin();

    FPoints ToPoints() const;

private:
    FLimits _limits;
    FPolygon _border;
    FPolygons _holes;

};

}

#endif //TRIVIS_CORE_GEOM_POLY_MAP_H_
