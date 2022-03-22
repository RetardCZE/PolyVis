/**
 * File:    simple_intersection.h
 *
 * Date:    07.08.2008
 * Author:  Jan Faigl
 *
 */

#ifndef UTILS_SIMPLE_INTERSECTION_H_
#define UTILS_SIMPLE_INTERSECTION_H_

#include <vector>
#include <cmath>

namespace intersection {

    template<int v>
    struct Int2Type {
        enum {
            value = v
        };
    };

/// ----------------------------------------------------------------------------
/// @brief  Implementation of intersection without improper intersection.
/// ----------------------------------------------------------------------------
    template<class T, bool kWithImproperIntersection = false>
    class Intersection {
    public:

        static constexpr double kEpsStd = 1e-6;
        static constexpr double kEpsStrict = 1e-7;

        enum Orientation {
            kLeftSide = 1, kCollinear = 0, kRightSide = -1
        };

        static bool OnLeftSide(const T &a, const T &b, const T &c) {
            return Wind(a, b, c) == kLeftSide;
        }

        static bool OnRightSide(const T &a, const T &b, const T &c) {
            return Wind(a, b, c) == kRightSide;
        }

        static double SquaredDistance(const T &a, const T &b) {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        static bool Equal(const double &a, const double &b) {
            return (fabs(a - b) < kEpsStrict);
        }

        static Orientation Wind(const T &a, const T &b, const T &c) {
            double w = ((a.y - b.y) * (c.x - b.x) - (c.y - b.y) * (a.x - b.x));
            // Need to allow for small math errors seen with "gcc -O2 -mcpu=i686 -ffast-math".
            return (w > kEpsStd) ? kLeftSide : ((w < -kEpsStd) ? kRightSide : kCollinear);
        }

        static T ZeroRound(const T &x) {
            return fabs(x) < kEpsStrict ? 0.0 : x;
        }

        static Orientation WindZeroRound(const T &a, const T &b, const T &c) {
            double w = (ZeroRound(a.y - b.y) * ZeroRound(c.x - b.x) - ZeroRound(c.y - b.y) * ZeroRound(a.x - b.x));
            // Need to allow for small math errors seen with "gcc -O2 -mcpu=i686 -ffast-math".
            return (w > kEpsStd) ? kLeftSide : ((w < -kEpsStd) ? kRightSide : kCollinear);
        }

        static double WindD(const T &a, const T &b, const T &c) {
            return ((a.y - b.y) * (c.x - b.x) - (c.y - b.y) * (a.x - b.x));
        }

        static double Area2(const T &a, const T &b, const T &c) {
            return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        }

    protected:

        static int InBetween(const T &a, const T &b, const T &c, Int2Type<false>) {
            if (!Equal(a.x, b.x)) return (((a.x < c.x) && (c.x < b.x)) || ((b.x < c.x) && (c.x < a.x)));
            else return (((a.y < c.y) && (c.y < b.y)) || ((b.y < c.y) && (c.y < a.y)));
        }

        static int InBetween(const T &a, const T &b, const T &c, Int2Type<true>) {
            if (!Equal(a.x, b.x)) return (((a.x <= c.x) && (c.x <= b.x)) || ((b.x <= c.x) && (c.x <= a.x)));
            else return (((a.y <= c.y) && (c.y <= b.y)) || ((b.y <= c.y) && (c.y <= a.y)));
        }

        static int InBetween(const T &a, const T &b, const T &c) {
            return InBetween(a, b, c, Int2Type<kWithImproperIntersection>());
        }

    public:

        static bool Xor(bool x, bool y) {
            return !x ^ !y;
        }

        /// ----------------------------------------------------------------------------
        /// Proper intersection.
        /// ----------------------------------------------------------------------------
        static bool Intersect(const T &a, const T &b, const T &c, const T &d, Int2Type<false>) {
            if (Collinear(a, b, c) || Collinear(a, b, d) || Collinear(c, d, a) || Collinear(c, d, b))
                return false;
            else
                return Xor(Wind(a, b, c) == kLeftSide, Wind(a, b, d) == kLeftSide)
                       && Xor(Wind(c, d, a) == kLeftSide, Wind(c, d, b) == kLeftSide);
        }

        /// ----------------------------------------------------------------------------
        /// Improper intersection.
        /// ----------------------------------------------------------------------------
        static bool Intersect(const T &a, const T &b, const T &c, const T &d, Int2Type<true>) {
            if (Intersect(a, b, c, d, Int2Type<false>())
                || OnSegment(a, b, c) || OnSegment(a, b, d) || OnSegment(c, d, a) || OnSegment(c, d, b))
                return true;
            else return false;
        }

        static bool Intersect(const T &a, const T &b, const T &c, const T &d) {
            return Intersect(a, b, c, d, Int2Type<kWithImproperIntersection>());
        }

        /// ----------------------------------------------------------------------------
        /// Return if intersection is improper.
        /// ----------------------------------------------------------------------------
        static bool Intersect(const T &a, const T &b, const T &c, const T &d, bool &improper, Int2Type<false>) {
            if (Collinear(a, b, c) || Collinear(a, b, d) || Collinear(c, d, a) || Collinear(c, d, b)) return false;
            else
                return Xor(Wind(a, b, c) == kLeftSide, Wind(a, b, d) == kLeftSide)
                       && Xor(Wind(c, d, a) == kLeftSide, Wind(c, d, b) == kLeftSide);
        }

        static bool Intersect(const T &a, const T &b, const T &c, const T &d, bool &improper, Int2Type<true>) {
            if (Intersect(a, b, c, d, Int2Type<false>())) return true;
            else if (OnSegment(a, b, c) || OnSegment(a, b, d) || OnSegment(c, d, a) || OnSegment(c, d, b)) {
                improper = true;
                return true;
            } else return false;
        }

        static bool Intersect(const T &a, const T &b, const T &c, const T &d, bool &improper) {
            improper = false;
            return Intersect(a, b, c, d, improper, Int2Type<kWithImproperIntersection>());
        }

        /// ----------------------------------------------------------------------------
        /// @brief Collinear
        ///
        /// @param a
        /// @param b
        /// @param c
        /// @param d
        ///
        /// @return true if segments (a,b) and (c,d) are Collinear that is a,b,c and a,b,d are Collinear
        /// ----------------------------------------------------------------------------
        static bool Collinear(const T &a, const T &b, const T &c, const T &d) {
            int a_abc;
            int a_abd;
            a_abc = Wind(a, b, c);
            a_abd = Wind(a, b, d);
            return (a_abc == 0 && a_abd == 0);
        }

        /// ----------------------------------------------------------------------------
        /// @brief Collinear
        ///
        /// @param a
        /// @param b
        /// @param c
        ///
        /// @return true if a,b,c are Collinear
        /// ----------------------------------------------------------------------------
        static bool Collinear(const T &a, const T &b, const T &c) {
            return Wind(a, b, c) == 0;
        }

        /// ----------------------------------------------------------------------------
        /// @brief AreCollinearPointsOrderedAlongLine
        ///
        /// @param a
        /// @param b
        /// @param c
        ///
        /// @return true of b lies between a and c, or b == a or b == c
        /// ----------------------------------------------------------------------------
        static bool AreCollinearPointsOrderedAlongLine(const T &a, const T &b, const T &c) {
            return InBetween(a, c, b);
        }

        /// ----------------------------------------------------------------------------
        /// @brief Collinear
        ///
        /// @param a
        /// @param b
        /// @param c
        ///
        /// @return true if a,b,c are collinear and c lies inside segment defined by a and b
        /// ----------------------------------------------------------------------------
        static bool CollinearSegment(const T &a, const T &b, const T &c) {
            return Wind(a, b, c) == 0 && InBetween(a, b, c);
        }

        /// ----------------------------------------------------------------------------
        /// @brief OnSegment
        ///
        /// @param a
        /// @param b
        /// @param c
        ///
        /// @return true if point c lines on segment (a,b)
        /// ----------------------------------------------------------------------------
        static bool OnSegment(const T &a, const T &b, const T &c) {
            if (Collinear(a, b, c)) {
                if (Equal(a.x, b.x)) {
                    return
                            ((a.y <= c.y && c.y <= b.y)) ||
                            ((a.y >= c.y && c.y >= b.y));
                } else {
                    return
                            ((a.x <= c.x && c.x <= b.x)) ||
                            ((a.x >= c.x && c.x >= b.x));
                }
            } else
                return false;
        }

        /// ----------------------------------------------------------------------------
        /// @brief OnSegment
        ///
        /// @param a
        /// @param b
        /// @param c
        /// @param wind (Wind of a,b, c)
        ///
        /// @return true if point c lines on segment (a,b) same as above, but use already computed Wind
        /// ----------------------------------------------------------------------------
        static bool OnSegment(const T &a, const T &b, const T &c, int wind) {
            if (wind == 0) {
                if (Equal(a.x, b.x)) return ((a.y <= c.y && c.y <= b.y)) || ((a.y >= c.y && c.y >= b.y));
                else return ((a.x <= c.x && c.x <= b.x)) || ((a.x >= c.x && c.x >= b.x));
            } else return false;
        }

        /// ----------------------------------------------------------------------------
        /// @brief InCone
        ///
        /// @param a0
        /// @param a1
        /// @param a2
        /// @param b
        ///
        /// @return true if b is in cone of a0, a1, a2
        /// ----------------------------------------------------------------------------
        static bool InCone(const T &a0, const T &a1, const T &a2, const T &b) {
            int m = Wind(b, a0, a1);
            int p = Wind(b, a1, a2);
            if (Wind(a0, a1, a2) > 0) return (m >= 0 && p >= 0); // convex at a1
            else return (m >= 0 || p >= 0); // reflex at a1
        }

        /// ----------------------------------------------------------------------------
        /// @brief Between based on algorithm ParallelInt from Computation Geomery in C
        ///
        /// @param a
        /// @param b
        /// @param c
        ///
        /// @return
        /// ----------------------------------------------------------------------------
        static bool Between(const T &a, const T &b, const T &c) {
            if (a.x != b.x) return (((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x)));
            else return (((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y)));
        }

        /// ----------------------------------------------------------------------------
        /// @brief ParallelIntersection based on algorithm ParallelInt from Computation Geomery in C
        ///
        /// @param a
        /// @param b
        /// @param c
        /// @param d
        /// @param p
        ///
        /// @return
        /// ----------------------------------------------------------------------------
        static char ParallelIntersection(const T &a, const T &b, const T &c, const T &d, T &p) {
            if (!Collinear(a, b, c)) return '0';
            else if (Between(a, b, c)) {
                p = c;
                return 'e';
            } else if (Between(a, b, d)) {
                p = d;
                return 'e';
            } else if (Between(c, d, a)) {
                p = a;
                return 'e';
            } else if (Between(c, d, b)) {
                p = b;
                return 'e';
            } else return '0';
        }

        /// ----------------------------------------------------------------------------
        /// @brief SegmentIntersection based on algorithm SegSegInt from Computation Geomery in C
        ///
        /// @param a segment(a,b)
        /// @param b
        /// @param c segment(c,d)
        /// @param d
        /// @param p intersection point
        ///
        /// @return '1' proper intersection, '0' no intersection
        ///         'e' edge intersection, 'v' vertex intersection
        /// ----------------------------------------------------------------------------
        static char SegmentIntersection(const T &a, const T &b, const T &c, const T &d, T &p) {
            char code = '0';
            double den = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) + c.x * (a.y - b.y);
            // if (den == 0.0)
            if (fabs(den) < kEpsStrict) code = ParallelIntersection(a, b, c, d, p);
            else {
                double num = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y);
                // if ((num == 0.0) || (num == den))
                if ((fabs(num) < kEpsStrict) || (fabs(num - den) < kEpsStrict)) {
                    code = 'v';
                }
                double s = num / den;
                num = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y));
                // if ((num == 0.0) || (num == den))
                if ((fabs(num) < kEpsStrict) || (fabs(num - den) < kEpsStrict)) {
                    code = 'v';
                }
                double t = num / den;
                if ((0.0 < s) && (s < 1.0) && (0.0 < t) && (t < 1.0)) {
                    code = '1';
                } else if (((0.0 > s) || (s > 1.0)) && ((0.0 > t) || (t > 1.0))) {
                    code = '0';
                }
                p.x = a.x + s * (b.x - a.x);
                p.y = a.y + s * (b.y - a.y);
            }
            return code;
        }

        static bool SegmentIntersection(const T &a, const T &b, const T &c, const T &d) {
            double den = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) + c.x * (a.y - b.y);
            if (fabs(den) < kEpsStrict) {
                if (!Collinear(a, b, c)) return false;
                else if (Between(a, b, c)) return true;
                else if (Between(a, b, d)) return true;
                else if (Between(c, d, a)) return true;
                else if (Between(c, d, b)) return true;
                else return false;
            } else {
                double s = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y) / den;
                double t = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y)) / den;
                if (((0.0 > s) || (s > 1.0)) && ((0.0 > t) || (t > 1.0))) return false;
                else return true;
            }
        }

        /// ----------------------------------------------------------------------------
        /// @brief SegmentIntersection based on algorithm SegSegInt from Computation Geomery in C
        ///
        /// @param a ray(a,b)
        /// @param b
        /// @param c segment(c,d)
        /// @param d
        /// @param p intersection point
        ///
        /// @return '1' proper intersection, '0' no intersection
        ///         'e' edge intersection, 'v' vertex intersection
        /// ----------------------------------------------------------------------------
        static char RaySegmentIntersection(const T &a, const T &b, const T &c, const T &d, T &p) {
            double s, t;
            double num, den;
            char code = '0';
            den = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) + c.x * (a.y - b.y);
            if (fabs(den) < kEpsStrict) code = ParallelIntersection(a, b, c, d, p);
            else {
                num = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y);
                if ((fabs(num) < kEpsStrict) || (fabs(num - den) < kEpsStrict)) code = 'v';
                s = num / den;
                num = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y));
                if ((fabs(num) < kEpsStrict) || (fabs(num - den) < kEpsStrict)) code = 'v';
                t = num / den;
                if ((0.0 < s) && (0.0 < t) && (t < 1.0)) code = '1';
                else if ((0.0 > s) && ((0.0 > t) || (t > 1.0))) code = '0';
                p.x = a.x + s * (b.x - a.x);
                p.y = a.y + s * (b.y - a.y);
            }
            return code;
        }

        static bool LineLineIntersect(const T &p1, const T &p2, const T &p3, const T &p4, T &pa, T &pb) {
            T p13, p43, p21;
            double d1343, d4321, d1321, d4343, d2121;
            double num, den;
            double mua, mub;
            p13.x = p1.x - p3.x;
            p13.y = p1.y - p3.y;
            p43.x = p4.x - p3.x;
            p43.y = p4.y - p3.y;
            if (fabs(p43.x) < kEpsStrict && fabs(p43.y) < kEpsStrict) return false;
            p21.x = p2.x - p1.x;
            p21.y = p2.y - p1.y;
            if (fabs(p21.x) < kEpsStrict && fabs(p21.y) < kEpsStrict) return false;
            d1343 = p13.x * p43.x + p13.y * p43.y;
            d4321 = p43.x * p21.x + p43.y * p21.y;
            d1321 = p13.x * p21.x + p13.y * p21.y;
            d4343 = p43.x * p43.x + p43.y * p43.y;
            d2121 = p21.x * p21.x + p21.y * p21.y;
            den = d2121 * d4343 - d4321 * d4321;
            if (fabs(den) < kEpsStrict) return false;
            num = d1343 * d4321 - d1321 * d4343;
            mua = num / den;
            mub = (d1343 + d4321 * mua) / d4343;
            pa.x = p1.x + mua * p21.x;
            pa.y = p1.y + mua * p21.y;
            pb.x = p3.x + mub * p43.x;
            pb.y = p3.y + mub * p43.y;
            return true;
        }

        // IsLeft(): tests if a point is Left|On|Right of an infinite line.
        //    Input:  three points P0, P1, and P2
        //    Return: >0 for P2 left of the line through P0 and P1
        //            =0 for P2 on the line
        //            <0 for P2 right of the line
        //    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
        //TODO  check int vs double return type
        static double IsLeft(const T &P0, const T &P1, const T &P2) {
            return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
        }

        /// ----------------------------------------------------------------------------
        /// @brief GetPointLineProjections find point p as projection of c to line (a,b)
        ///
        /// @param c
        /// @param a
        /// @param b
        /// @param p
        /// ----------------------------------------------------------------------------
        static void GetPointLineProjections(const T &c, const T &a, const T &b, T &p) {
            const double r_num = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
            const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            const double r = r_num / r_den;
            p.x = a.x + r * (b.x - a.x);
            p.y = a.y + r * (b.y - a.y);
        }

        /// ----------------------------------------------------------------------------
        /// @brief point_line_distance return distance of point c from line a,b
        ///
        /// @param c
        /// @param a
        /// @param b
        ///
        /// @return
        /// ----------------------------------------------------------------------------
        static double PointLineSquaredDistance(const T &c, const T &a, const T &b) {
            const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            const double s = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y);
            return s * s / r_den;
        }

        /// ----------------------------------------------------------------------------
        /// @brief PointSegmentSquaredDistance return distance of point c from segment a,b
        ///
        /// @param c
        /// @param a
        /// @param b
        ///
        /// @return
        /// ----------------------------------------------------------------------------
        static double PointSegmentSquaredDistance(const T &c, const T &a, const T &b) {
            const double r_num = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
            const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            const double r = r_num / r_den;
            double ret;
            if ((r >= 0) && (r <= 1)) { // segment distance is line distance
                const double s = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y);
                ret = s * s / r_den;
            } else { // segment distance is distance from c to a or from c to b
                const double dist1 = SquaredDistance(c, a);
                const double dist2 = SquaredDistance(c, b);
                ret = dist1 < dist2 ? dist1 : dist2;
            }
            return ret;
        }

        /// ----------------------------------------------------------------------------
        /// @brief PointSegmentSquaredDistance
        ///
        /// same as above, but return indication endpoint indication and the point itself
        /// endpoint indication:
        /// 's' inside segment
        /// 'a' at the a point
        /// 'b' at the b point
        /// ----------------------------------------------------------------------------
        static double PointSegmentSquaredDistance(const T &c, const T &a, const T &b, char &endpoint, T &p) {
            const double r_num = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
            const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            const double r = r_num / r_den;
            double ret;
            if ((r >= 0) && (r <= 1)) { // segment distance is line distance
                const double s = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y);
                p.x = a.x + r * (b.x - a.x);
                p.y = a.y + r * (b.y - a.y);
                ret = s * s / r_den;
                endpoint = 's';
            } else { // segment distance is distance from c to a or from c to b
                const double dist1 = SquaredDistance(c, a);
                const double dist2 = SquaredDistance(c, b);
                if (dist1 < dist2) {
                    ret = dist1;
                    p = a;
                    endpoint = 'a';
                } else {
                    ret = dist2;
                    p = b;
                    endpoint = 'b';
                }
            }
            return ret;
        }

        /// ----------------------------------------------------------------------------
        /// PointInPolygonWN(): winding number test for a point in a poly
        ///      Input:   p = a point,
        ///               poly[] = vertex points of a poly poly[n+1] with poly[n]=poly[0]
        ///      Return:  wn = the winding number (=0 only if p is outside poly[])
        static int PointInPolygonWN(const T &p, const std::vector<T> &poly) {
            int wn = 0; // the winding number counter
            const int n = poly.size();
            // loop through all edges of the poly
            for (int i = 0; i < n; i++) { // edge from poly[i] to poly[i+1]
                if (poly[i].y <= p.y) { // start y <= p.y
                    if (poly[i + 1].y > p.y) // an upward crossing
                        if (IsLeft(poly[i], poly[i + 1], p) > 0) // p left of edge
                            ++wn; // have a valid up intersect
                } else { // start y > p.y (no test needed)
                    if (poly[i + 1].y <= p.y) // a downward crossing
                        if (IsLeft(poly[i], poly[i + 1], p) < 0)  // p right of edge
                            --wn; // have a valid down intersect
                }
            }
            return wn;
        }

        /// ----------------------------------------------------------------------------
        /// PointInPolygonCN(): crossing number test for a point in a poly
        ///      Input:   p = a point,
        ///               poly[] = vertex points of a poly poly[n+1] with poly[n]=poly[0]
        ///      Return:  0 = outside, 1 = inside
        /// This code is patterned after [Franklin, 2000]
        static int PointInPolygonCN(const T &p, const std::vector<T> &poly) {
            int cn = 0;    // the crossing number counter
            const int n = poly.size();

            // loop through all edges of the poly
            for (int i = 0; i < n; i++) {    // edge from poly[i] to poly[i+1]
                if (((poly[i].y <= p.y) && (poly[i + 1].y > p.y))    // an upward crossing
                    || ((poly[i].y > p.y) && (poly[i + 1].y <= p.y))) { // a downward crossing
                    // compute the actual edge-ray intersect x-coordinate
                    double vt = (double) 1.0 * (p.y - poly[i].y) / (poly[i + 1].y - poly[i].y);
                    if (p.x < poly[i].x + vt * (poly[i + 1].x - poly[i].x)) // p.x < intersect
                        ++cn;   // a valid crossing of y=p.y right of p.x
                }
            }
            return (cn & 1);    // 0 if even (out), and 1 if odd (in)
        }

        /// This seems be correct form the above listed version (be aware that poly[n] = poly[0], only for the multiple component).
        static int PointInPolygon(const T &p, const std::vector<T> &poly) {
            int c = 0;
            const int n = poly.size();
            int j = n - 1;
            for (int i = 0; i < n; j = i++) {
                if (((poly[i].y > p.y) != (poly[j].y > p.y))
                    && (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
                    c = !c;
            }
            return c;
        }

    };

}

#endif //UTILS_SIMPLE_INTERSECTION_H_