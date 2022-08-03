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

using namespace edgevis;

robust_geom::Orientation robust_geom::Orient(const Point &a, const Point &b, const Point &c) {
    auto result = predicates::orient2d(&a.x, &b.x, &c.x);
    if (result > 0.0) return robust_geom::Orientation::kLeftTurn;
    if (result < 0.0) return robust_geom::Orientation::kRightTurn;
    return robust_geom::Orientation::kCollinear;
}

robust_geom::Orientation robust_geom::OrientWithEps(const Point &a, const Point &b, const Point &c, double eps) {
    auto result = predicates::orient2d(&a.x, &b.x, &c.x);
    if (std::abs(result) <= eps) {
        return robust_geom::Orientation::kCollinear;
    }
    if (result > 0.0) return robust_geom::Orientation::kLeftTurn;
    return robust_geom::Orientation::kRightTurn;
}

bool robust_geom::TurnsLeft(const Point &a, const Point &b, const Point &c) {
    return robust_geom::Orient(a, b, c) == robust_geom::Orientation::kLeftTurn;
}

bool robust_geom::TurnsRight(const Point &a, const Point &b, const Point &c) {
    return robust_geom::Orient(a, b, c) == robust_geom::Orientation::kRightTurn;
}

bool robust_geom::Collinear(const Point &a, const Point &b, const Point &c) {
    return robust_geom::Orient(a, b, c) == robust_geom::Orientation::kCollinear;
}

bool robust_geom::TurnsLeftWithEps(const Point &a, const Point &b, const Point &c, double eps) {
    return robust_geom::OrientWithEps(a, b, c, eps) == robust_geom::Orientation::kLeftTurn;
}

bool robust_geom::TurnsRightWithEps(const Point &a, const Point &b, const Point &c, double eps) {
    return robust_geom::OrientWithEps(a, b, c, eps) == robust_geom::Orientation::kRightTurn;
}

bool robust_geom::CollinearWithEps(const Point &a, const Point &b, const Point &c, double eps) {
    return robust_geom::OrientWithEps(a, b, c, eps) == robust_geom::Orientation::kCollinear;
}

char robust_geom::PointTriangleRelation(const Point &q, const Point &a, const Point &b, const Point &c) {
    robust_geom::Orientation o_abq = robust_geom::Orient(a, b, q);
    if (o_abq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    robust_geom::Orientation o_bcq = robust_geom::Orient(b, c, q);
    if (o_bcq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    robust_geom::Orientation o_caq = robust_geom::Orient(c, a, q);
    if (o_caq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    if (o_abq == robust_geom::Orientation::kCollinear) {
        if (o_bcq == robust_geom::Orientation::kCollinear) {
            return 'B';
        }
        if (o_caq == robust_geom::Orientation::kCollinear) {
            return 'A';
        }
        return 'a';
    }
    if (o_bcq == robust_geom::Orientation::kCollinear) {
        if (o_caq == robust_geom::Orientation::kCollinear) {
            return 'C';
        }
        return 'b';
    }
    if (o_caq == robust_geom::Orientation::kCollinear) {
        return 'c';
    }
    return '1';
}

char robust_geom::PointTriangleRelationWithEps(const Point &q, const Point &a, const Point &b, const Point &c, double eps) {
    robust_geom::Orientation o_abq = robust_geom::OrientWithEps(a, b, q, eps);
    if (o_abq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    robust_geom::Orientation o_bcq = robust_geom::OrientWithEps(b, c, q, eps);
    if (o_bcq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    robust_geom::Orientation o_caq = robust_geom::OrientWithEps(c, a, q, eps);
    if (o_caq == robust_geom::Orientation::kRightTurn) {
        return '0';
    }
    if (o_abq == robust_geom::Orientation::kCollinear) {
        if (o_bcq == robust_geom::Orientation::kCollinear) {
            return 'B';
        }
        if (o_caq == robust_geom::Orientation::kCollinear) {
            return 'A';
        }
        return 'a';
    }
    if (o_bcq == robust_geom::Orientation::kCollinear) {
        if (o_caq == robust_geom::Orientation::kCollinear) {
            return 'C';
        }
        return 'b';
    }
    if (o_caq == robust_geom::Orientation::kCollinear) {
        return 'c';
    }
    return '1';
}

bool robust_geom::IsPointInCone(const Point &q, const Point &a, const Point &b, const Point &c) {
    if (!robust_geom::TurnsRight(a, b, c)) {
        // convex at b
        return !robust_geom::TurnsRight(q, a, b) && !robust_geom::TurnsRight(q, b, c);
    }
    // reflex at b
    return !robust_geom::TurnsRight(q, a, b) || !robust_geom::TurnsRight(q, b, c);
}
