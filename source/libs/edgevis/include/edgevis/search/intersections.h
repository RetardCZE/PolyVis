/**
 * File:   intersections.h
 *
 * Date:   14.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "edgevis/structs/point.h"
using namespace edgevis;
namespace robust_geom {
bool LineLineIntersectionNotCollinear(const Point &a, const Point &b, const Point &c, const Point &d, Point &p);
/**
 *
 * @param a IN: 1st endpoint of segment a-b.
 * @param b IN: 2nd endpoint of segment a-b.
 * @param c IN: 1st endpoint of segment c-d.
 * @param d IN: 2nd endpoint of segment c-d.
 * @param p OUT: 1st endpoint of the intersection segment, if a-b and c-d intersect in a segment (see Code 's'); else intersection of a-b and c-d if it exists.
 * @param q OUT: 2nd endpoint of the intersection segment, if a-b and c-d intersect in a segment (see Code 's').
 * @return Code (char), see bellow:
 *      Code 'c': Segments are collinear and do not intersect.
 *      Code 'V': Segments are collinear and share a single endpoint p.
 *      Code 's': Segments are collinear and intersect in segment p-q.
 *      Code '0': Segments are NOT collinear and do not intersect.
 *      Code 'v': Segments are NOT collinear and share a single endpoint p.
 *      Code 'i': Segments are NOT collinear and intersect in p, which is an endpoint of either a-b or c-d.
 *      Code '1': Segments are NOT collinear and have a PROPER intersection in p.
 *      Code 'e': Segments are NOT collinear and SHOULD intersect but the intersection was not found (ERROR STATE).
 *  Segments are collinear: 'c', 'V', 's'.
 *  Segments are NOT collinear: '0', 'v', 'i', '1', 'e'.
 *  Segments intersect (intersection found): 'V', 's', 'v', 'i', '1'.
 *  Segments intersect (intersection NOT found): 'e'.
 *  Segments DO NOT intersect: 'c', '0'.
 *  Intersection NOT found: 'c', '0', 'e'.
 */

}

