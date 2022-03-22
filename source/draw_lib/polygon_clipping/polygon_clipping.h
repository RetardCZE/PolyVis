/**
 * File:    polygon_clipping.h
 *
 * Date:    19.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef POLYGON_CLIPPING_POLYGON_CLIPPING_H_
#define POLYGON_CLIPPING_POLYGON_CLIPPING_H_

#include "../geom/geom.h"

#include "clipper/clipper.hpp"

namespace polygon_clipping {

    using CInt = ClipperLib::cInt;
    using CPoint = ClipperLib::IntPoint;
    using CPolygon = ClipperLib::Path;
    using CPolygons = ClipperLib::Paths;
    using CClipType = ClipperLib::ClipType;
    using CClipper = ClipperLib::Clipper;
    using COffset = ClipperLib::ClipperOffset;

    static constexpr double kDefaultMultiplier = 32768.0;
    static constexpr double kDefaultDenominator = kDefaultMultiplier;

    template<typename T>
    inline auto Geom2Clipper(const geom::Point<T> &p, const double &multiplier = kDefaultMultiplier) {
        return CPoint(static_cast<CInt>(p.x * multiplier), static_cast<CInt>(p.y * multiplier));
    }

    template<typename T>
    auto Geom2Clipper(const geom::Polygon<T> &poly, const double &multiplier = kDefaultMultiplier) {
        CPolygon c_poly;
        for (const auto &p : poly) c_poly.push_back(std::move(Geom2Clipper(p, multiplier)));
        return std::move(c_poly);
    }

    template<typename T>
    auto Geom2Clipper(const geom::Polygons<T> &polys, const double &multiplier = kDefaultMultiplier) {
        CPolygons c_polys;
        for (const auto &poly : polys) c_polys.push_back(std::move(Geom2Clipper(poly, multiplier)));
        return std::move(c_polys);
    }

    template<typename T>
    inline auto Clipper2Geom(const CPoint &c_p, const double &denominator = kDefaultDenominator) {
        return geom::Point<T>(static_cast<T>(c_p.X / denominator), static_cast<T>(c_p.Y / denominator));
    }

    template<typename T>
    auto Clipper2Geom(const CPolygon &c_poly, const double &denominator = kDefaultDenominator) {
        geom::Polygon<T> poly;
        for (const auto &c_p : c_poly) poly.push_back(std::move(Clipper2Geom<T>(c_p, denominator)));
        return std::move(poly);
    }

    template<typename T>
    auto Clipper2Geom(const CPolygons &c_polys, const double &denominator = kDefaultDenominator) {
        geom::Polygons<T> polys;
        for (const auto &c_poly : c_polys) polys.push_back(std::move(Clipper2Geom<T>(c_poly, denominator)));
        return std::move(polys);
    }

    inline void Clip(CClipType ct, const CPolygon &subj, const CPolygon &clip, CPolygons &solution) {
        CClipper c;
        c.AddPath(subj, ClipperLib::ptSubject, true);
        c.AddPath(clip, ClipperLib::ptClip, true);
        c.Execute(ct, solution, ClipperLib::pftNonZero);
        c.Clear();
    }

    inline void Clip(CClipType ct, const CPolygons &subj, const CPolygon &clip, CPolygons &solution) {
        CClipper c;
        c.AddPaths(subj, ClipperLib::ptSubject, true);
        c.AddPath(clip, ClipperLib::ptClip, true);
        c.Execute(ct, solution, ClipperLib::pftNonZero);
        c.Clear();
    }

    inline void Clip(CClipType ct, const CPolygon &subj, const CPolygons &clip, CPolygons &solution) {
        CClipper c;
        c.AddPath(subj, ClipperLib::ptSubject, true);
        c.AddPaths(clip, ClipperLib::ptClip, true);
        c.Execute(ct, solution, ClipperLib::pftNonZero);
        c.Clear();
    }

    inline void Clip(CClipType ct, const CPolygons &subj, const CPolygons &clip, CPolygons &solution) {
        CClipper c;
        c.AddPaths(subj, ClipperLib::ptSubject, true);
        c.AddPaths(clip, ClipperLib::ptClip, true);
        c.Execute(ct, solution, ClipperLib::pftNonZero);
        c.Clear();
    }

    inline auto Orientation(const CPolygon &polygon) {
        return ClipperLib::Orientation(polygon);
    }

    inline auto Area(const CPolygon &polygon) {
        return ClipperLib::Area(polygon);
    }

    double Area(const CPolygons &polygons);

    inline auto SimplifyPolygons(CPolygons &polygons) {
        return ClipperLib::SimplifyPolygons(polygons);
    }

    inline auto CleanPolygon(CPolygon &polygon,
                             const double &distance = 0.001,
                             const double &multiplier = kDefaultMultiplier) {
        return ClipperLib::CleanPolygon(polygon, distance * multiplier);
    }

    inline auto CleanPolygons(CPolygons &polygons,
                              const double &distance = 0.001,
                              const double &multiplier = kDefaultMultiplier) {
        return ClipperLib::CleanPolygons(polygons, distance * multiplier);
    }

    inline bool PointInOrOnPolygon(const CPoint &point, const CPolygon &polygon) {
        int code = ClipperLib::PointInPolygon(point, polygon);
        if (code == +1 || code == -1) return true;
        else return false;
    }

    inline bool PointInPolygon(const CPoint &point, const CPolygon &polygon) {
        int code = ClipperLib::PointInPolygon(point, polygon);
        if (code == +1) return true;
        else return false;
    }

    inline bool PointOnPolygon(const CPoint &point, const CPolygon &polygon) {
        int code = ClipperLib::PointInPolygon(point, polygon);
        if (code == -1) return true;
        else return false;
    }

    inline void Offset(CPolygons &polygons, const double &offset, const double &multiplier = kDefaultMultiplier) {
        ClipperLib::ClipperOffset co;
        co.AddPaths(polygons, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(polygons, offset * multiplier);
    }

    inline void OffsetConvex(CPolygon &polygon, const double &offset, const double &multiplier = kDefaultMultiplier) {
        ClipperLib::ClipperOffset co;
        co.AddPath(polygon, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        CPolygons solution;
        co.Execute(solution, offset * multiplier);
        if (solution.empty()) polygon.clear();
        else polygon = solution[0];
    }

}

#endif //POLYGON_CLIPPING_POLYGON_CLIPPING_H_
