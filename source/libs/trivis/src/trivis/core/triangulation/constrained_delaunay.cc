/**
 * File:   constrained_delaunay.cc
 *
 * Date:   23.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "trivis/core/triangulation/constrained_delaunay.h"

#include <cstring>

extern "C"
{
#define VOID int
#define REAL double
#include "trivis/core/libs/triangle/triangle.h"
}

#include "trivis/core/geom/robust_geometry.h"

using namespace trivis::core;
using namespace trivis::core::geom;
using namespace trivis::core::triangulation;

void triangulation::TriangulateMapConstrainedDelaunay(
    const PolyMap &map,
    TriMesh &mesh,
    FPolygons &triangles
) {

    int num_points = static_cast<int>(map.border().size());
    for (const auto &hole: map.holes()) {
        num_points += static_cast<int>(hole.size());
    }
    int num_holes = static_cast<int>(map.holes().size());

    struct triangulateio in = {}, out = {}, vorout = {};

    // in. allocations.
    in.numberofpoints = num_points;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.numberofpointattributes = 0;
    in.pointmarkerlist = (int *) nullptr;

    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *) nullptr;

    in.numberofholes = (int) num_holes;
    in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));

    // Insert points, segments and holes.
    int idx_points = 0, idx_segments = 0, idx_holes = 0;
    int num_points_inserted = 0;
    for (int i = 0; i < 1 + num_holes; ++i) {
        const auto &polygon = (i == 0) ? map.border() : map.holes()[i - 1];
        int idx_first_point_of_current = num_points_inserted;
        for (int j = 0; j < polygon.size(); ++j, ++num_points_inserted) {
            // Insert points:
            in.pointlist[idx_points++] = polygon[j].x;
            in.pointlist[idx_points++] = polygon[j].y;
            // Insert segments:
            if (j != polygon.size() - 1) { // if not the last segment
                in.segmentlist[idx_segments++] = num_points_inserted;
                in.segmentlist[idx_segments++] = num_points_inserted + 1;
            } else {
                in.segmentlist[idx_segments++] = num_points_inserted;
                in.segmentlist[idx_segments++] = idx_first_point_of_current;
            }
        }
        // Insert holes:
        if (i > 0) {
            auto hole_point = FindAnyPointInNonConvexPolygon(polygon, true);
            in.holelist[idx_holes++] = hole_point.x;
            in.holelist[idx_holes++] = hole_point.y;
        }
    }

    // out. allocations.
    out.pointlist = (REAL *) nullptr;
    out.pointmarkerlist = (int *) nullptr;
    out.trianglelist = (int *) nullptr;
    out.segmentlist = (int *) nullptr;
    out.segmentmarkerlist = (int *) nullptr;
    out.edgelist = (int *) nullptr;
    vorout.pointlist = (REAL *) nullptr;
    vorout.edgelist = (int *) nullptr;
    vorout.normlist = (REAL *) nullptr;

    std::string options = "pQzve"; // see https://www.cs.cmu.edu/~quake/triangle.switch.html

    // Solve.
    char *c_options = new char[options.length() + 1];
    std::strcpy(c_options, options.c_str());
    triangulate(c_options, &in, &out, &vorout);

    // Fill triangles.
    triangles.reserve(out.numberoftriangles);
    for (int i = 0, j = 0; i < out.numberoftriangles * 3; i += 3, j++) {
        geom::Polygon<double> triangle;
        triangle.emplace_back(out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]);
        triangle.emplace_back(out.pointlist[out.trianglelist[i + 1] * 2], out.pointlist[out.trianglelist[i + 1] * 2 + 1]);
        triangle.emplace_back(out.pointlist[out.trianglelist[i + 2] * 2], out.pointlist[out.trianglelist[i + 2] * 2 + 1]);
        // Orient the triangle counter-clockwise.
        if (geom::OrientationClockwise(triangle)) {
            geom::ChangeOrientation(triangle);
        }
        triangles.push_back(std::move(triangle));
    }

    // Fill graph nodes.
    mesh.nodes.reserve(out.numberofpoints);
    for (int i = 0; i < 2 * out.numberofpoints; i += 2) {
        TriNode node;
        node.point = geom::MakePoint(out.pointlist[i], out.pointlist[i + 1]);
        mesh.nodes.push_back(std::move(node));
    }

    // Fill graph edges.
    mesh.nodes.reserve(out.numberofedges);
    for (int i = 0, j = 0; i < out.numberofedges * 2; i += 2, ++j) {
        TriEdge edge;
        edge.nodes[0] = out.edgelist[i];
        edge.nodes[1] = out.edgelist[i + 1];
        mesh.edges.push_back(edge);
        mesh.nodes[out.edgelist[i]].edges.push_back(j);
        mesh.nodes[out.edgelist[i + 1]].edges.push_back(j);
    }

    // Fill graph triangles.
    mesh.triangles.reserve(out.numberoftriangles);
    for (int i = 0, j = 0; i < out.numberoftriangles * 3; i += 3, ++j) {
        TriTriangle triangle{};
        triangle.nodes[0] = out.trianglelist[i];
        triangle.nodes[1] = out.trianglelist[i + 1];
        triangle.nodes[2] = out.trianglelist[i + 2];
        triangle.edges = {-1, -1, -1};
        mesh.triangles.push_back(triangle);
        mesh.nodes[out.trianglelist[i]].triangles.push_back(j);
        mesh.nodes[out.trianglelist[i + 1]].triangles.push_back(j);
        mesh.nodes[out.trianglelist[i + 2]].triangles.push_back(j);
    }

    // Voronoi edges <=> triangles.
    for (int i = 0, j = 0; i < vorout.numberofedges * 2; i += 2, ++j) {
        int tri_id = vorout.edgelist[i];
        if (tri_id != -1) {
            for (int &e_id: mesh.triangles[tri_id].edges) {
                if (e_id == -1) {
                    e_id = j;
                    break;
                }
            }
            mesh.edges[j].triangles.push_back(tri_id);
            mesh.edges[j].opposites.push_back(OppositeNode(mesh, tri_id, j));
        }
        tri_id = vorout.edgelist[i + 1];
        if (tri_id != -1) {
            for (int &e_id: mesh.triangles[tri_id].edges) {
                if (e_id == -1) {
                    e_id = j;
                    break;
                }
            }
            mesh.edges[j].triangles.push_back(tri_id);
            mesh.edges[j].opposites.push_back(OppositeNode(mesh, tri_id, j));
        }
    }

    // Sort structures.
    SortStructures(mesh);

    // Cleaning up:
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.segmentmarkerlist);
    free(in.holelist);
    free(out.pointlist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(vorout.pointlist);
    free(vorout.edgelist);
    free(vorout.normlist);
}

void triangulation::TriangulateMapConstrainedDelaunay(
    const FPolygons &regions,
    FPolygons &triangles
) {
    FPoints points_no_duplicates;
    std::vector<int> points_orig_to_points_no_duplicates;
    {   // detect duplicate points
        std::vector<std::pair<FPoint, int>> point_id_list;
        for (int i = 0, id = 0; i < regions.size(); ++i) {
            for (int j = 0; j < regions[i].size(); ++j, ++id) {
                point_id_list.emplace_back(regions[i][j], id);
            }
        }
        std::sort(point_id_list.begin(), point_id_list.end(), [](const std::pair<FPoint, int> &a, const std::pair<FPoint, int> &b) {
            return a.first.x < b.first.x || (a.first.x == b.first.x && a.first.y < b.first.y);
        });
        points_no_duplicates.reserve(point_id_list.size());
        points_orig_to_points_no_duplicates.resize(point_id_list.size());
        for (int i = 0, cnt = -1; i < point_id_list.size(); ++i) {
            const auto &p = point_id_list[i].first;
            int id = point_id_list[i].second;
            if (i == 0 || p != point_id_list[i - 1].first) {
                points_no_duplicates.push_back(p);
                ++cnt;
            }
            points_orig_to_points_no_duplicates[id] = cnt;
        }
    }

    int num_pnt = static_cast<int>(points_no_duplicates.size());
    int num_hol = 0;
    int num_seg = 0;
    std::vector<bool> is_hole(regions.size(), false);
    for (int i = 0; i < regions.size(); ++i) {
        const auto &region = regions[i];
        num_seg += static_cast<int>(region.size());
        if (geom::OrientationClockwise(region)) {
            is_hole[i] = true;
            ++num_hol;
        }
    }

    struct triangulateio in = {}, out = {};

    // in. allocations.
    in.numberofpoints = num_pnt;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.numberofpointattributes = 0;
    in.pointmarkerlist = (int *) nullptr;

    in.numberofsegments = num_seg;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *) nullptr;

    in.numberofholes = (int) num_hol;
    if (num_hol > 0) {
        in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));
    }

    // Insert points.
    for (int i = 0, cnt_pnt = 0; i < points_no_duplicates.size(); ++i) {
        in.pointlist[cnt_pnt++] = points_no_duplicates[i].x;
        in.pointlist[cnt_pnt++] = points_no_duplicates[i].y;
    }

    // Insert segments and holes.
    for (int i = 0, cnt_seg = 0, cnt_hol = 0, cnt_pnt = 0; i < regions.size(); ++i) {
        const auto &region = regions[i];
        int cnt_pnt_0 = cnt_pnt;
        for (int j = 0; j < region.size(); ++j, ++cnt_pnt) {
            // Insert segments:
            if (j + 1 != region.size()) { // if not the last segment
                in.segmentlist[cnt_seg++] = points_orig_to_points_no_duplicates[cnt_pnt];
                in.segmentlist[cnt_seg++] = points_orig_to_points_no_duplicates[cnt_pnt + 1];
            } else {
                in.segmentlist[cnt_seg++] = points_orig_to_points_no_duplicates[cnt_pnt];
                in.segmentlist[cnt_seg++] = points_orig_to_points_no_duplicates[cnt_pnt_0];
            }
        }
        // Insert holes:
        if (is_hole[i]) {
            auto hole_point = FindAnyPointInNonConvexPolygon(region, true);
            in.holelist[cnt_hol++] = hole_point.x;
            in.holelist[cnt_hol++] = hole_point.y;
        }
    }

    // // Insert points, segments and holes.
    // int idx_points = 0, idx_segments = 0, idx_holes = 0;
    // int num_points_inserted = 0;
    // for (int i = 0; i < regions.size(); ++i) {
    //     const auto &polygon = regions[i];
    //     int idx_first_point_of_current = num_points_inserted;
    //     for (int j = 0; j < polygon.size(); ++j, ++num_points_inserted) {
    //         // Insert points:
    //         in.pointlist[idx_points++] = polygon[j].x;
    //         in.pointlist[idx_points++] = polygon[j].y;
    //         // Insert segments:
    //         if (j != polygon.size() - 1) { // if not the last segment
    //             in.segmentlist[idx_segments++] = num_points_inserted;
    //             in.segmentlist[idx_segments++] = num_points_inserted + 1;
    //         } else {
    //             in.segmentlist[idx_segments++] = num_points_inserted;
    //             in.segmentlist[idx_segments++] = idx_first_point_of_current;
    //         }
    //     }
    //     // Insert holes:
    //     if (is_hole[i]) {
    //         auto hole_point = FindAnyPointInNonConvexPolygon(polygon, true);
    //         in.holelist[idx_holes++] = hole_point.x;
    //         in.holelist[idx_holes++] = hole_point.y;
    //     }
    // }

    // out. allocations.
    out.pointlist = (REAL *) nullptr;
    out.pointmarkerlist = (int *) nullptr;
    out.trianglelist = (int *) nullptr;
    out.segmentlist = (int *) nullptr;
    out.segmentmarkerlist = (int *) nullptr;
    out.edgelist = (int *) nullptr;

    std::string options = "pQz"; // see https://www.cs.cmu.edu/~quake/triangle.switch.html

    // Solve.
    char *c_options = new char[options.length() + 1];
    std::strcpy(c_options, options.c_str());
    triangulate(c_options, &in, &out, (struct triangulateio *) nullptr);

    // Fill triangles.
    triangles.reserve(out.numberoftriangles);
    for (int i = 0, j = 0; i < out.numberoftriangles * 3; i += 3, j++) {
        geom::Polygon<double> triangle;
        triangle.emplace_back(out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]);
        triangle.emplace_back(out.pointlist[out.trianglelist[i + 1] * 2], out.pointlist[out.trianglelist[i + 1] * 2 + 1]);
        triangle.emplace_back(out.pointlist[out.trianglelist[i + 2] * 2], out.pointlist[out.trianglelist[i + 2] * 2 + 1]);
        // Orient the triangle counter-clockwise.
        if (geom::OrientationClockwise(triangle)) {
            geom::ChangeOrientation(triangle);
        }
        triangles.push_back(std::move(triangle));
    }

    // Cleaning up:
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.segmentmarkerlist);
    free(in.holelist);
    free(out.pointlist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
}
