/**
 * File:   intersections.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 * Edited by Jakub Rosol. (Added SegmentSegmentIntersectionGeneral for EdgeVis project)
 */
#include "edgevis/helpers/intersections.h"

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

    uint8_t
    SegmentSegmentIntersectionGeneral(const Point &a, const Point &b, const Point &c, const Point &d, Point &p,
                                      bool robust) {
        robustOrientation aOri = Orient(c, d, a, robust);
        robustOrientation bOri = Orient(c, d, b, robust);
        robustOrientation cOri = Orient(a, b, c, robust);
        robustOrientation dOri = Orient(a, b, d, robust);
        // this is passed only if lines are intersecting and not fully collinear (4 points in line)
        if(cOri != dOri && aOri != bOri){
            // if one of segment point is collinear, it means it has to be the intersection
            if (cOri == robustOrientation::kCollinear) {
                p = c;
                return 2;
            }
            if (dOri == robustOrientation::kCollinear) {
                p = d;
                return 2;
            }
            // same as 1st check but for 2nd intersection
            if (aOri == robustOrientation::kCollinear) {
                p = a;
                return 1;
            }
            if (bOri == robustOrientation::kCollinear) {
                p = b;
                return 1;
            }
            // if intersection is not defined by segment defining points it has to be calculated
            bool check = LineLineIntersectionNotCollinear(a, b, c, d, p);
            if (check) {
                return 0;
            } else {
                return 3;
            }
        }
        return 4;
        /*
         * only if all special cases are checked we move to common intersection calculation. That can be done with line line.
         */
    }

}