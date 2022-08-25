/**
 * File:   intersections.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */
#include "edgevis/search/intersections.h"

namespace edgevis {

/**
 * Assumption: 'abc' is not collinear AND 'abd' is not collinear.
 */

    bool LineLineIntersectionNotCollinear(const Point &a, const Point &b, const Point &c, const Point &d, Point &p) {
        double den = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) + c.x * (a.y - b.y);
        // den should never be zero if the assumption is satisfied
        if (den == 0.0) {
            // just to be sure (should not happen!)
            return false;
        }
        double num = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y);
        double split = num / den;
        p.x = a.x + split * (b.x - a.x);
        p.y = a.y + split * (b.y - a.y);
        return true;
    }

    uint8_t LineSegmentIntersectionGeneral(const Point &a, const Point &b, const Point &c, const Point &d, Point &p) {
        robustOrientation cOri = Orient(a, b, c);
        robustOrientation dOri = Orient(a, b, d);
        //std::cout << static_cast<int>(cOri) << " | " << static_cast<int>(dOri) << std::endl;
        if (cOri == dOri) {
            /* specific cases
             * */
            if (cOri == robustOrientation::kCollinear) {
                /*
                 * Segment is part of a->b line. This should not occur.
                 */
                return 3;
            }
            if (cOri == robustOrientation::kLeftTurn) {
                return 1;
            }
            if (cOri == robustOrientation::kRightTurn) {
                return 2;
            }
        }
        if (cOri == robustOrientation::kCollinear) {
            p = c;
            if (dOri == robustOrientation::kLeftTurn) {
                return 5;
            } else {
                return 6;
            }
        }
        if (dOri == robustOrientation::kCollinear) {
            p = d;
            if (cOri == robustOrientation::kLeftTurn) {
                return 5;
            } else {
                return 6;
            }
        }
        /*
         * only if all special cases are checked we move to common intersection calculation. That can be done with line line.
         */
        bool check = LineLineIntersectionNotCollinear(a, b, c, d, p);
        if (check) {
            return 0;
        } else {
            return 4;
        }


    }

}