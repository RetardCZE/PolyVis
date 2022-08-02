/**
 * File:   robust_geometry.cc
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "edgevis/search/robust_geometry.h"

#include "edgevis/libs/predicates/predicates.h"

#include <cassert>

using namespace trivis::core;
using namespace trivis::core::geom;

Orientation geom::Orient(const FPoint &a, const FPoint &b, const FPoint &c) {
    auto result = predicates::orient2d(&a.x, &b.x, &c.x);
    if (result > 0.0) return Orientation::kLeftTurn;
    if (result < 0.0) return Orientation::kRightTurn;
    return Orientation::kCollinear;
}

Orientation geom::OrientWithEps(const FPoint &a, const FPoint &b, const FPoint &c, double eps) {
    auto result = predicates::orient2d(&a.x, &b.x, &c.x);
    if (std::abs(result) <= eps) {
        return Orientation::kCollinear;
    }
    if (result > 0.0) return Orientation::kLeftTurn;
    return Orientation::kRightTurn;
}

bool geom::TurnsLeft(const FPoint &a, const FPoint &b, const FPoint &c) {
    return Orient(a, b, c) == Orientation::kLeftTurn;
}

bool geom::TurnsRight(const FPoint &a, const FPoint &b, const FPoint &c) {
    return Orient(a, b, c) == Orientation::kRightTurn;
}

bool geom::Collinear(const FPoint &a, const FPoint &b, const FPoint &c) {
    return Orient(a, b, c) == Orientation::kCollinear;
}

bool geom::TurnsLeftWithEps(const FPoint &a, const FPoint &b, const FPoint &c, double eps) {
    return OrientWithEps(a, b, c, eps) == Orientation::kLeftTurn;
}

bool geom::TurnsRightWithEps(const FPoint &a, const FPoint &b, const FPoint &c, double eps) {
    return OrientWithEps(a, b, c, eps) == Orientation::kRightTurn;
}

bool geom::CollinearWithEps(const FPoint &a, const FPoint &b, const FPoint &c, double eps) {
    return OrientWithEps(a, b, c, eps) == Orientation::kCollinear;
}

char geom::PointTriangleRelation(const FPoint &q, const FPoint &a, const FPoint &b, const FPoint &c) {
    Orientation o_abq = Orient(a, b, q);
    if (o_abq == Orientation::kRightTurn) {
        return '0';
    }
    Orientation o_bcq = Orient(b, c, q);
    if (o_bcq == Orientation::kRightTurn) {
        return '0';
    }
    Orientation o_caq = Orient(c, a, q);
    if (o_caq == Orientation::kRightTurn) {
        return '0';
    }
    if (o_abq == Orientation::kCollinear) {
        if (o_bcq == Orientation::kCollinear) {
            return 'B';
        }
        if (o_caq == Orientation::kCollinear) {
            return 'A';
        }
        return 'a';
    }
    if (o_bcq == Orientation::kCollinear) {
        if (o_caq == Orientation::kCollinear) {
            return 'C';
        }
        return 'b';
    }
    if (o_caq == Orientation::kCollinear) {
        return 'c';
    }
    return '1';
}

char geom::PointTriangleRelationWithEps(const FPoint &q, const FPoint &a, const FPoint &b, const FPoint &c, double eps) {
    Orientation o_abq = OrientWithEps(a, b, q, eps);
    if (o_abq == Orientation::kRightTurn) {
        return '0';
    }
    Orientation o_bcq = OrientWithEps(b, c, q, eps);
    if (o_bcq == Orientation::kRightTurn) {
        return '0';
    }
    Orientation o_caq = OrientWithEps(c, a, q, eps);
    if (o_caq == Orientation::kRightTurn) {
        return '0';
    }
    if (o_abq == Orientation::kCollinear) {
        if (o_bcq == Orientation::kCollinear) {
            return 'B';
        }
        if (o_caq == Orientation::kCollinear) {
            return 'A';
        }
        return 'a';
    }
    if (o_bcq == Orientation::kCollinear) {
        if (o_caq == Orientation::kCollinear) {
            return 'C';
        }
        return 'b';
    }
    if (o_caq == Orientation::kCollinear) {
        return 'c';
    }
    return '1';
}

bool geom::IsPointInCone(const FPoint &q, const FPoint &a, const FPoint &b, const FPoint &c) {
    if (!TurnsRight(a, b, c)) {
        // convex at b
        return !TurnsRight(q, a, b) && !TurnsRight(q, b, c);
    }
    // reflex at b
    return !TurnsRight(q, a, b) || !TurnsRight(q, b, c);
}

FPolygon geom::RemoveDuplicatePoints(const FPolygon &polygon) {
    geom::FPolygon ret;
    int n = static_cast<int>(polygon.size());
    int i = 0;
    ret.reserve(n);
    while (true) {
        ret.push_back(polygon[i]);
        int j = i;
        bool same;
        while (j < n && (same = (polygon[++j] == polygon[i])));
        if (j == n - 1) {
            if (!same) {
                i = j;
            }
            if (polygon[i] != polygon[0]) {
                ret.push_back(polygon[i]);
            }
            break;
        }
        i = j;
    }
    return ret;
}

FPolygon geom::RemoveCollinearPoints(const FPolygon &polygon) {
    geom::FPolygon ret;
    int n = static_cast<int>(polygon.size());
    int u = n - 1, v = 0, w = 1;
    while (v < n) {
        if (!Collinear(polygon[u], polygon[v], polygon[w])) {
            ret.push_back(polygon[v]);
        }
        ++u, ++v, ++w;
        if (u == n) u = 0;
        if (w == n) w = 0;
    }
    return ret;
}

FPoint geom::FindAnyPointInNonConvexPolygon(const FPolygon &polygon, bool clockwise) {
    assert(!polygon.empty());

    int n = static_cast<int>(polygon.size());

    // Deal with a point, segment, and triangle.
    if (n <= 3) {
        FPoint p = polygon[0];
        for (int i = 1; i < polygon.size(); ++i) {
            p = p + polygon[i];
        }
        p = p / n;
        return p;
    }

    // 1) Identify a convex vertex v
    int idx_u = n - 1, idx_v = 0, idx_w = 1;
    while (idx_v < n) {
        if (clockwise && TurnsRight(polygon[idx_u], polygon[idx_v], polygon[idx_w])) {
            break;
        }
        if (!clockwise && TurnsLeft(polygon[idx_u], polygon[idx_v], polygon[idx_w])) {
            break;
        }
        ++idx_u, ++idx_v, ++idx_w;
        if (idx_u == n) idx_u = 0;
        if (idx_w == n) idx_w = 0;
    }
    if (clockwise) {
        std::swap(idx_u, idx_w);
    }
    const auto &u = polygon[idx_u];
    const auto &v = polygon[idx_v];
    const auto &w = polygon[idx_w];

    // 2) For each other vertex p do
    int idx_q = -1;
    double min_sq_distance_to_v = std::numeric_limits<double>::max();
    for (int idx_p = 0; idx_p < polygon.size(); ++idx_p) {
        const auto &p = polygon[idx_p];
        // 2a) if p is inside triangle uvw,
        if (idx_p != idx_u && idx_p != idx_v && idx_p != idx_w && TurnsLeft(u, v, p) && TurnsLeft(v, w, p) && !TurnsRight(w, u, p)) {
            // compute squared distance to p.
            auto sq_distance_to_v = p.SquaredDistanceTo(v);
            // 2b) Save point p as point q if distance is a new min.
            if (sq_distance_to_v < min_sq_distance_to_v) {
                min_sq_distance_to_v = sq_distance_to_v;
                idx_q = idx_p;
            }
        }
    }

    if (idx_q == -1) {
        // 3) If no point is inside, return centroid of uvw.
        return geom::MakePoint((u.x + v.x + w.x) / 3.0, (u.y + v.y + w.y) / 3.0);
    } else {
        // 4) Else if some point inside, qv is internal: return its midpoint.
        return geom::MakePoint((polygon[idx_q].x + v.x) / 2.0, (polygon[idx_q].y + v.y) / 2.0);
    }
}
