#pragma once
#include "OG_poly/structs/mesh.h"
#include "OG_poly/structs/point.h"

namespace OG_poly
{
    //Throw snell ray, and return the point on the edge of mesh2Pass
    Point projectRay(Point start, Point middle, int poly2Leave,int poly2Pass, Mesh* mesh);
    //Find the snell ray from start point reaching target point, return the point where snellray meets the middle edge
    Point computeAngle(Point start, Point target,int poly2Leave, int poly2Pass, Mesh* mesh);
}
