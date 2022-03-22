/**
 * File:    utils.h
 *
 * Date:    26.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef GEOM_UTILS_H_
#define GEOM_UTILS_H_

#include "./geom.h"

#include <limits>

namespace geom {

    template<typename T>
    Polygon <T> MakeRegularPolygon(const Point <T> &center, const T &radius, unsigned n) {
        Polygon<T> poly;
        double angle = 2.0 * M_PI / n;
        for (int i = 0; i < n; ++i) {
            auto alpha = i * angle;
            poly.emplace_back(static_cast<T>(center.x + radius * sin(alpha)),
                              static_cast<T>(center.y + radius * cos(alpha)));
        }
        return std::move(poly);
    }

    template<typename T>
    void ComputeLimits(const Polygon <T> &polygon, T &x_min, T &x_max, T &y_min, T &y_max) {
        x_min = std::numeric_limits<T>::max();
        y_min = std::numeric_limits<T>::max();
        x_max = std::numeric_limits<T>::lowest();
        y_max = std::numeric_limits<T>::lowest();
        for (const auto &p : polygon) {
            if (p.x < x_min) x_min = p.x;
            if (p.y < y_min) y_min = p.y;
            if (x_max < p.x) x_max = p.x;
            if (y_max < p.y) y_max = p.y;
        }
    }

    template<typename T>
    void ComputeLimits(const Polygons <T> &polygons, T &x_min, T &x_max, T &y_min, T &y_max) {
        x_min = std::numeric_limits<T>::max();
        y_min = std::numeric_limits<T>::max();
        x_max = std::numeric_limits<T>::lowest();
        y_max = std::numeric_limits<T>::lowest();
        for (const auto &poly : polygons) {
            for (const auto &p : poly) {
                if (p.x < x_min) x_min = p.x;
                if (p.y < y_min) y_min = p.y;
                if (x_max < p.x) x_max = p.x;
                if (y_max < p.y) y_max = p.y;
            }
        }
    }

    template<typename T>
    void ComputeLimits(const CMap <T> &circles, T &x_min, T &x_max, T &y_min, T &y_max) {
        x_min = std::numeric_limits<T>::max();
        y_min = std::numeric_limits<T>::max();
        x_max = std::numeric_limits<T>::lowest();
        y_max = std::numeric_limits<T>::lowest();
        for (const Circle<T> &circle : circles) {
            Point<T> center = circle.center;
            T r = circle.radius;
            T x = center.x;
            T y = center.y;
            if ((x - r) < x_min) x_min = (x - r);
            if ((y - r) < y_min) y_min = ((y - r));
            if (x_max < (x + r)) x_max = (x + r);
            if (y_max < (y + r)) y_max = ((y + r));
        }
    }
}

#endif //GEOM_UTILS_H_
