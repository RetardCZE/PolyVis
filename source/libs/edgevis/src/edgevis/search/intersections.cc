/**
 * File:   intersections.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "edgevis/search/intersections.h"

#include "edgevis/search/robust_geometry.h"

using namespace edgevis;

/**
 * Assumption: 'abc' is not collinear AND 'abd' is not collinear.
 */
bool  robust_geom::LineLineIntersectionNotCollinear(const Point &a, const Point &b, const Point &c, const Point &d, Point &p) {
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

