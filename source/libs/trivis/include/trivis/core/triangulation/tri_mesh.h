/**
 * File:   tri_mesh.h
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_TRIANGULATION_TRI_MESH_H_
#define TRIVIS_CORE_TRIANGULATION_TRI_MESH_H_

#include <array>

#include "trivis/core/geom/geom_types.h"

namespace trivis::core::triangulation {

struct TriNode {
    geom::FPoint point;
    std::vector<int> edges;
    std::vector<int> triangles;
};

struct TriEdge {
    std::array<int, 2> nodes;
    std::vector<int> triangles;
    std::vector<int> opposites;
    [[nodiscard]] bool is_obstacle() const { return triangles.size() == 1; }
};

struct TriTriangle {
    std::array<int, 3> edges;
    std::array<int, 3> nodes;
};

struct TriMesh {
    std::vector<TriNode> nodes;
    std::vector<TriEdge> edges;
    std::vector<TriTriangle> triangles;
    [[nodiscard]] const auto &point(int id) const { return nodes[id].point; }
};

[[nodiscard]] inline int OppositeNode(
    const TriMesh &mesh,
    int tri_id,
    int e_id
) {
    return mesh.triangles[tri_id].nodes[0] + mesh.triangles[tri_id].nodes[1] + mesh.triangles[tri_id].nodes[2] - mesh.edges[e_id].nodes[0] - mesh.edges[e_id].nodes[1];
}

void SortStructures(TriMesh &mesh);

}

#endif //TRIVIS_CORE_TRIANGULATION_TRI_MESH_H_
