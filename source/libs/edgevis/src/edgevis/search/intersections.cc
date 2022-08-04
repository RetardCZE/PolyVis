/**
 * File:   intersections.cc
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "edgevis/search/intersections.h"

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

uint8_t robust_geom::LineSegmentIntersectionGeneral(const Point &a, const Point &b, const Point &c, const Point &d, Point &p){
    robust_geom::Orientation cOri = robust_geom::Orient(a, b , c);
    robust_geom::Orientation dOri = robust_geom::Orient(a, b, d);
    if(cOri == dOri){
        /* specific cases
         * */
        if(cOri == robust_geom::Orientation::kCollinear){
            /*
             * Segment is part of a->b line. This should not occur.
             */
            return 3;
        }
        if(cOri == robust_geom::Orientation::kLeftTurn){
            return 1;
        }
        if(cOri == robust_geom::Orientation::kRightTurn){
            return 2;
        }
    }
    if(cOri == robust_geom::Orientation::kCollinear){
        p = c;
        return 0;
    }
    if(dOri == robust_geom::Orientation::kCollinear){
        p = d;
        return 0;
    }
    /*
     * only if all special cases are checked we move to common intersection calculation. That can be done with line line.
     */
    bool check = robust_geom::LineLineIntersectionNotCollinear(a, b, c, d, p);
    if(check){
        return 0;
    }else{
        return 4;
    }


}

