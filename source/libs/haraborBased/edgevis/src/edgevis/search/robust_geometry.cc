/**
 * File:   robust_geometry.cc
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */
#include "edgevis/search/robust_geometry.h"

#include <cassert>

namespace edgevis {

    robustOrientation Orient(const Point &a, const Point &b, const Point &c) {
        auto result = predicates::orient2d(&a.x, &b.x, &c.x);
        if (result > 0.0) return robustOrientation::kLeftTurn;
        if (result < 0.0) return robustOrientation::kRightTurn;
        return robustOrientation::kCollinear;
    }

    robustOrientation OrientWithEps(const Point &a, const Point &b, const Point &c, double eps) {
        auto result = predicates::orient2d(&a.x, &b.x, &c.x);
        if (std::abs(result) <= eps) {
            return robustOrientation::kCollinear;
        }
        if (result > 0.0) return robustOrientation::kLeftTurn;
        return robustOrientation::kRightTurn;
    }

    bool TurnsLeft(const Point &a, const Point &b, const Point &c) {
        return Orient(a, b, c) == robustOrientation::kLeftTurn;
    }

    bool TurnsRight(const Point &a, const Point &b, const Point &c) {
        return Orient(a, b, c) == robustOrientation::kRightTurn;
    }

    bool Collinear(const Point &a, const Point &b, const Point &c) {
        return Orient(a, b, c) == robustOrientation::kCollinear;
    }

    bool TurnsLeftWithEps(const Point &a, const Point &b, const Point &c, double eps) {
        return OrientWithEps(a, b, c, eps) == robustOrientation::kLeftTurn;
    }

    bool TurnsRightWithEps(const Point &a, const Point &b, const Point &c, double eps) {
        return OrientWithEps(a, b, c, eps) == robustOrientation::kRightTurn;
    }

    bool CollinearWithEps(const Point &a, const Point &b, const Point &c, double eps) {
        return OrientWithEps(a, b, c, eps) == robustOrientation::kCollinear;
    }

    char PointTriangleRelation(const Point &q, const Point &a, const Point &b, const Point &c) {
        robustOrientation o_abq = Orient(a, b, q);
        if (o_abq == robustOrientation::kRightTurn) {
            return '0';
        }
        robustOrientation o_bcq = Orient(b, c, q);
        if (o_bcq == robustOrientation::kRightTurn) {
            return '0';
        }
        robustOrientation o_caq = Orient(c, a, q);
        if (o_caq == robustOrientation::kRightTurn) {
            return '0';
        }
        if (o_abq == robustOrientation::kCollinear) {
            if (o_bcq == robustOrientation::kCollinear) {
                return 'B';
            }
            if (o_caq == robustOrientation::kCollinear) {
                return 'A';
            }
            return 'a';
        }
        if (o_bcq == robustOrientation::kCollinear) {
            if (o_caq == robustOrientation::kCollinear) {
                return 'C';
            }
            return 'b';
        }
        if (o_caq == robustOrientation::kCollinear) {
            return 'c';
        }
        return '1';
    }

    char PointTriangleRelationWithEps(const Point &q, const Point &a, const Point &b, const Point &c, double eps) {
        robustOrientation o_abq = OrientWithEps(a, b, q, eps);
        if (o_abq == robustOrientation::kRightTurn) {
            return '0';
        }
        robustOrientation o_bcq = OrientWithEps(b, c, q, eps);
        if (o_bcq == robustOrientation::kRightTurn) {
            return '0';
        }
        robustOrientation o_caq = OrientWithEps(c, a, q, eps);
        if (o_caq == robustOrientation::kRightTurn) {
            return '0';
        }
        if (o_abq == robustOrientation::kCollinear) {
            if (o_bcq == robustOrientation::kCollinear) {
                return 'B';
            }
            if (o_caq == robustOrientation::kCollinear) {
                return 'A';
            }
            return 'a';
        }
        if (o_bcq == robustOrientation::kCollinear) {
            if (o_caq == robustOrientation::kCollinear) {
                return 'C';
            }
            return 'b';
        }
        if (o_caq == robustOrientation::kCollinear) {
            return 'c';
        }
        return '1';
    }

    bool IsPointInCone(const Point &q, const Point &a, const Point &b, const Point &c) {
        if (!TurnsRight(a, b, c)) {
            // convex at b
            return !TurnsRight(q, a, b) && !TurnsRight(q, b, c);
        }
        // reflex at b
        return !TurnsRight(q, a, b) || !TurnsRight(q, b, c);
    }
}