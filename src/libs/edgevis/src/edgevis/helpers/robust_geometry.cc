/**
 * File:   robust_geometry.cc
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 * note: Edited by Jakub Rosol. (Only reduced to needed function)
 */
#include "edgevis/helpers/robust_geometry.h"

#include <cassert>

namespace edgevis {

    robustOrientation Orient(const Point &a, const Point &b, const Point &c, bool robust) {
        if(!robust) {
            {   // without robust predicates
                const double cross = (b - a) * (c - b);
                if (std::abs(cross) < EPSILON) {
                    return robustOrientation::kCollinear;
                } else if (cross > 0) {
                    return robustOrientation::kLeftTurn;
                } else {
                    return robustOrientation::kRightTurn;
                }
            }
        }else{
            auto result = predicates::orient2d(&a.x, &b.x, &c.x);
            if (result > 0.0) return robustOrientation::kLeftTurn;
            if (result < 0.0) return robustOrientation::kRightTurn;
            return robustOrientation::kCollinear;
        }
    }

}