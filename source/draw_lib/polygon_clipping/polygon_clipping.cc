/**
 * File:    polygon_clipping.cc
 *
 * Date:    19.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */




#include "polygon_clipping.h"

double polygon_clipping::Area(const CPolygons &polygons) {
    double area = 0.0;
    for (const auto &polygon : polygons) area += ClipperLib::Area(polygon);
    return area;
}

