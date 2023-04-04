/**
 * File:   robust_geometry.h
 *
 * Date:   22.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */
#pragma once
#include "edgevis/structs/point.h"
#include "edgevis/structs/polygon.h"
#include "edgevis/libs/predicates/predicates.h"

namespace edgevis{
        enum class robustOrientation : int {
            kRightTurn = -1,
            kCollinear = 0,
            kLeftTurn = 1
        };

        robustOrientation Orient(const Point &a, const Point &b, const Point &c, bool robust);
}