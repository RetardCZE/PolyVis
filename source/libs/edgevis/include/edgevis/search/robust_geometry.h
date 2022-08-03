/**
 * File:   robust_geometry.h
 *
 * Date:   22.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "edgevis/structs/point.h"
#include "edgevis/structs/polygon.h"

using namespace edgevis;
namespace robust_geom {

enum class Orientation : int {
    kRightTurn = -1,
    kCollinear = 0,
    kLeftTurn = 1
};

Orientation Orient(const Point &a, const Point &b, const Point &c);

Orientation OrientWithEps(const Point &a, const Point &b, const Point &c, double eps);

bool TurnsLeft(const Point &a, const Point &b, const Point &c);

bool TurnsRight(const Point &a, const Point &b, const Point &c);

bool Collinear(const Point &a, const Point &b, const Point &c);

bool TurnsLeftWithEps(const Point &a, const Point &b, const Point &c, double eps);

bool TurnsRightWithEps(const Point &a, const Point &b, const Point &c, double eps);

bool CollinearWithEps(const Point &a, const Point &b, const Point &c, double eps);

/**
 *
 * Returns true if c is on the extension of the ray starting in b in the direction b-->a, i.e., if (b-a)*(c-b) >= 0, and false otherwise.
 *  <---------a-------b
 *      FALSE | FALSE | TRUE
 * Exact implementation, works only if a, b, c are EXACTLY collinear.
 */
inline bool Extends(const Point &a, const Point &b, const Point &c) {
    return (a.x == b.x) ? ((a.y <= b.y) ? (b.y <= c.y) : (b.y >= c.y)) : ((a.x <= b.x) ? (b.x <= c.x) : (b.x >= c.x));
}

/**
 *  <---------a------b--------->
 *      FALSE | TRUE | FALSE
 *
 * Exact implementation, works only if a, b, c are EXACTLY collinear.
 */
inline bool IsBetween(const Point &a, const Point &b, const Point &c) {
    return !Extends(a, b, c) && !Extends(b, a, c);
}

/**
 *
 * @param q IN: The query point.
 * @param a IN: 1st point defining the counter-clockwise triangle a-b-c.
 * @param b IN: 2nd point defining the counter-clockwise triangle a-b-c.
 * @param c IN: 3rd point defining the counter-clockwise triangle a-b-c.
 * @return Code (char), see bellow:
 *      Code 'a': Point q lies on segment a-b.
 *      Code 'b': Point q lies on segment b-c.
 *      Code 'c': Point q lies on segment c-a.
 *      Code 'A': Point q is equal to point a.
 *      Code 'B': Point q is equal to point b
 *      Code 'C': Point q is equal to point c.
 *      Code '1': Point q is inside triangle a-b-c but not on its border.
 *      Code '0': Point is outside of triangle a-b-c.
 *  Point q is strictly inside OR on the border of a-b-c: '1', 'a', 'b', 'c', 'A', 'B', 'C'
 *  Point q lies on the border of a-b-c: 'a', 'b', 'c', 'A', 'B', 'C'.
 *  Point q is equal to one of the triangle vertices:  'A', 'B', 'C'.
 */
char PointTriangleRelation(const Point &q, const Point &a, const Point &b, const Point &c);

char PointTriangleRelationWithEps(const Point &q, const Point &a, const Point &b, const Point &c, double eps);

bool IsPointInCone(const Point &q, const Point &a, const Point &b, const Point &c);

}
