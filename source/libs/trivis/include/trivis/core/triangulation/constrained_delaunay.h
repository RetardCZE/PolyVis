/**
 * File:   constrained_delaunay.h
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_TRIANGULATION_CONSTRAINED_DELAUNAY_H_
#define TRIVIS_CORE_TRIANGULATION_CONSTRAINED_DELAUNAY_H_

#include "trivis/core/geom/poly_map.h"

#include "trivis/core/triangulation/tri_mesh.h"

namespace trivis::core::triangulation {

void TriangulateMapConstrainedDelaunay(
    const geom::PolyMap &map,
    TriMesh &mesh,
    geom::FPolygons &triangles
);

void TriangulateMapConstrainedDelaunay(
    const geom::FPolygons &regions,
    geom::FPolygons &triangles
);

}

#endif //TRIVIS_CORE_TRIANGULATION_CONSTRAINED_DELAUNAY_H_
