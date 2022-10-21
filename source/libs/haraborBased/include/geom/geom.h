/**
 * File:    geom.h
 *
 * Date:    18.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef GEOM_GEOM_H_
#define GEOM_GEOM_H_

#include <cmath>
#include <vector>
#include <string>
#include <ostream>

namespace geom {

    template<typename T>
    struct Point {

        T x = T(0);
        T y = T(0);

        Point() = default;

        Point(const T &x, const T &y) : x(x), y(y) {}

        Point(const Point &p) : x(p.x), y(p.y) {}

        Point(Point &p) : x(p.x), y(p.y) {}

        Point &operator=(const Point &p) {
            if (this != &p) {
                x = p.x;
                y = p.y;
            }
            return *this;
        }

        Point &operator=(Point &&) noexcept = default;

        virtual ~Point() = default;

        Point operator+(const Point &p) const { return Point(x + p.x, y + p.y); }

        Point operator-(const Point &p) const { return Point(x - p.x, y - p.y); }

        Point operator*(T f) const { return Point(f * x, f * y); }

        Point operator/(T f) const { return Point(x / f, y / f); }

        bool operator==(const Point &p) const { return x == p.x && y == p.y; }

        bool operator!=(const Point &p) const { return !(x == p.x && y == p.y); }

        T NormSquared() const { return x * x + y * y; }

        T Norm() const { return std::sqrt(NormSquared()); }

        T SquaredDistanceTo(Point p) const { return (*this - p).NormSquared(); }

        T DistanceTo(Point p) const { return (*this - p).Norm(); }

        std::string ToString(const std::string &delimiter = " ") const {
            return std::to_string(x) + delimiter + std::to_string(y);
        }

        friend std::ostream &operator<<(std::ostream &os, const Point &p) {
            return os << std::to_string(p.x) << " " << std::to_string(p.y);
        }

    };

    template<typename T>
    inline auto MakePoint(const T &x, const T &y) { return Point<T>(x, y); }

    template<typename T>
    using Points = std::vector<Point<T>>;

    template<typename T>
    using Polygon = Points<T>;

    template<typename T>
    using Polygons = std::vector<Points<T>>;

    // Polygon map
    template<typename T>
    struct PolygonMap {
        Polygon<T> border;
        Polygons<T> obstacles;
    };

    template<typename T>
    struct Circle {

        T radius = T(0);
        Point<T> center = Point<T>();

        Circle() = default;

        Circle(const T &radius, const Point<T> &center) : radius(radius), center(center) {}

        Circle(const Circle &) = default;

        Circle(Circle
               &&) noexcept = default;

        Circle &operator=(const Circle &) = default;

        Circle &operator=(Circle &&) noexcept = default;

        virtual ~Circle() = default;

        Circle operator+(const Circle &c) const { return Circle(radius + c.radius, center + c.center); }

        Circle operator-(const Circle &c) const { return Circle(radius - c.radius, center - c.center); }

        bool operator==(const Circle &c) const { return radius == c.radius && center == c.center; }

        bool operator!=(const Circle &c) const { return !(radius == c.radius && center == c.center); }

        std::string ToString(const std::string &delimiter = " ") const {
            return std::to_string(radius) + delimiter + std::to_string(center.ToString());
        }

        friend std::ostream &operator<<(std::ostream &os, const Circle &c) {
            return os << "r: " << std::to_string(c.radius) << " " << "center: " << c.center.ToString();
        }

    };

    template<typename T>
    using CMap = std::vector<Circle<T>>;

    template<typename T>
    struct Position {

        T x = T(0);
        T y = T(0);
        T a = T(0);

        Position() = default;

        Position(const T &x, const T &y) : x(x), y(y) {}

        Position(const T &x, const T &y, const T &a) : x(x), y(y), a(a) {}

        Position(const Position &) = default;

        Position(Position &&) noexcept = default;

        Position &operator=(const Position &) = default;

        Position &operator=(Position &&) noexcept = default;

        virtual ~Position() = default;

        Position operator+(const Position &p) const { return Position(x + p.x, y + p.y, a + p.a); }

        Position operator-(const Position &p) const { return Position(x - p.x, y - p.y, a + p.a); }

        bool operator==(const Position &p) const { return x == p.x && y == p.y && a == p.a; }

        bool operator!=(const Position &p) const { return !(x == p.x && y == p.y && a == p.a); }

        Point<T> ToPoint() { return Point<T>(x, y); }

        [[nodiscard]] std::string ToString(const std::string &delimiter = " ") const {
            return std::to_string(x) + delimiter + std::to_string(y) + delimiter + std::to_string(a);
        }

        friend std::ostream &operator<<(std::ostream &os, const Position &p) {
            return os << std::to_string(p.x) << " " << std::to_string(p.y) << " " << std::to_string(p.a);
        }

    };

    template<typename T>
    inline auto MakePosition(const T &x, const T &y) { return Position<T>(x, y); }

    template<typename T>
    inline auto MakePosition(const T &x, const T &y, const T &a) { return Position<T>(x, y, a); }

    template<typename T>
    using Positions = std::vector<Position<T>>;

}

#endif //GEOM_GEOM_H_
