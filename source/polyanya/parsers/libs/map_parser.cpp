/**
 * File:    map_parser.cpp
 *
 * Date:   24.03.2021
 * Author:  Lukas Fanta
 * E-mail:  fantalukas2108@gmail.com
 *
 */

#include "map_parser.h"


void parsers::MapParser::convertMapToFade2DMesh(const string &filename, parsers::Fade2DMesh &fade2DMesh) {
    GEOM_FADE2D::Fade_2D dt;
    GEOM_FADE2D::Zone2 *traversable;
    std::vector<parsers::Fade2DPolygon> obstacles;

    traversable = fadeutils::create_traversable_zone_filename(filename, dt, obstacles);
    convertToFade2DMesh(dt, traversable, obstacles, fade2DMesh);
}

void parsers::MapParser::convertIstreamToFade2DMesh(std::istream &infile, parsers::Fade2DMesh &fade2DMesh) {
    GEOM_FADE2D::Fade_2D dt;
    GEOM_FADE2D::Zone2 *traversable;
    std::vector<parsers::Fade2DPolygon> obstacles;

    traversable = fadeutils::create_traversable_zone_istream(infile, dt, obstacles);
    convertToFade2DMesh(dt, traversable, obstacles, fade2DMesh);
//    printFade2DMesh(fade2DMesh,std::cout);
}

void parsers::MapParser::convertToFade2DMesh(const GEOM_FADE2D::Fade_2D &dt, GEOM_FADE2D::Zone2 *traversable,
                                             const std::vector<parsers::Fade2DPolygon> &obstacles,
                                             parsers::Fade2DMesh &fade2DMesh) {
    // Create local array
    vector<GEOM_FADE2D::Point2 *> localVertices;
    vector<GEOM_FADE2D::Triangle2 *> localTriangles;
    map<GEOM_FADE2D::Triangle2 *, int> triangleToIndex;

    // Show Zone and Fade_2D
    //    traversable->show("example_zone.ps",true,true);
    //    dt.show("example_fade2d.ps", true);

    // Initialise triangles
    traversable->getTriangles(localTriangles);
    for (int i = 0; i < (int) localTriangles.size(); i++) {
        triangleToIndex[localTriangles[i]] = i;
    }

    // Initialise vertices and neighbours
    dt.getVertexPointers(localVertices);
    fade2DMesh.vertices.resize(localVertices.size());
    for (int i = 0; i < (int) localVertices.size(); i++) {

        // Set vertices
        localVertices[i]->setCustomIndex(i);

        // Init Fade2DVertex struct
        parsers::Fade2DVertex fade2DVertex;

        typedef GEOM_FADE2D::TriangleAroundVertexIterator TAVI;
        TAVI start(localVertices[i]);
        bool first = true;
        for (TAVI it(start); (it != start) || first; ++it) {
            first = false;
            GEOM_FADE2D::Triangle2 *cur_triangle = *it;

            // triangle not in traversable area
            if (cur_triangle == nullptr || triangleToIndex.count(cur_triangle) == 0) {
                if (fade2DVertex.idxNeighVertices.empty() || fade2DVertex.idxNeighVertices.back() != -1) {
                    fade2DVertex.idxNeighVertices.push_back(-1);
                }
            } else {
                fade2DVertex.idxNeighVertices.push_back(triangleToIndex[cur_triangle]);
            }
        }

        // Most -1s should be removed already, but it is possible that
        // the first AND last element are -1s.
        assert(!fade2DVertex.idxNeighVertices.empty());
        if (fade2DVertex.idxNeighVertices.front() == -1 && fade2DVertex.idxNeighVertices.back() == -1) {
            fade2DVertex.idxNeighVertices.pop_back();
        }
        fade2DVertex.point = *localVertices[i];
        fade2DMesh.vertices[i] = fade2DVertex;
    }

    // Static arrays
    // Vertices 0 1 2
    const int vertexIndex[] = {0, 1, 2};

    // Triangles.
    // Go 2 0 1 in version 1, 1 2 0 in version 2.
    int v = 2;
    const int triangleIndex[] = {v == 1 ? 2 : 1, v == 1 ? 0 : 2, v == 1 ? 1 : 0};

    for (int i = 0; i < (int) localTriangles.size(); ++i) {
        parsers::Fade2DTriangle triangle;
        triangle.ptrTriangle = localTriangles[i];
        triangle.idxVertices.resize(3);

        // Save index of vertex in polygon (triangle)
        for (int j : vertexIndex) {
            triangle.idxVertices[j] = triangle.ptrTriangle->getCorner(j)->getCustomIndex();
        }

        // Save indexes of neighbouring polygons
        for (int j : triangleIndex) {
            GEOM_FADE2D::Triangle2 *cur_triangle = triangle.ptrTriangle->getOppositeTriangle(j);
            if (cur_triangle == nullptr || triangleToIndex.count(cur_triangle) == 0) {
                triangle.idxNeighTriangles.push_back(-1);
            } else {
                triangle.idxNeighTriangles.push_back(triangleToIndex[cur_triangle]);
            }
        }
        // Save triangle to mesh struct
        fade2DMesh.triangles.push_back(triangle);
    }
    fade2DMesh.obstacles = obstacles;
}

void
parsers::MapParser::readMergedMeshFromFade2DMesh(const parsers::Fade2DMesh &fade2DMesh,
                                                 parsers::MergedMesh &mergedMesh) {

    // Init arrays
    int V = fade2DMesh.vertices.size();
    int P = fade2DMesh.triangles.size();

    mergedMesh.mesh_vertices.resize(V);
    mergedMesh.mesh_polygons.resize(P);
    mergedMesh.polygon_unions = UnionFind(P);

    for (int i = 0; i < V; i++) {
        Vertex &v = mergedMesh.mesh_vertices[i];
        parsers::Fade2DVertex fade2DVertex = fade2DMesh.vertices[i];

        v.p.x = fade2DVertex.point.x();
        v.p.y = fade2DVertex.point.y();
        int neighbours = fade2DVertex.idxNeighVertices.size();

        v.num_polygons = neighbours;
        // Guaranteed to have 2 or more.
        ListNodePtr cur_node = nullptr;

        for (int j = 0; j < neighbours; j++) {
            int polygon_index = fade2DVertex.idxNeighVertices[j];

            if (polygon_index >= P) {
                cerr << "Got a polygon index of " << polygon_index << endl;
            }

            ListNodePtr new_node = makeNode(nullptr, polygon_index);

            if (j == 0) {
                cur_node = new_node;
                v.polygons = cur_node;
            } else {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = v.polygons;
    }

    for (int i = 0; i < P; i++) {
        Polygon &p = mergedMesh.mesh_polygons[i];

        parsers::Fade2DTriangle triangle = fade2DMesh.triangles[i];
        // Number of vertices of polygon (triangle) is always three
        int n = triangle.idxVertices.size();
        p.num_vertices = n;

        ListNodePtr cur_node = nullptr;
        for (int j = 0; j < n; j++) {
            // Index of vertex for given polygon
            int vertex_index = triangle.idxVertices[j];

            if (vertex_index >= V) {
                cerr << "Got a vertex index of " << vertex_index << endl;
            }

            ListNodePtr new_node = makeNode(nullptr, vertex_index);

            if (j == 0) {
                cur_node = new_node;
                p.vertices = cur_node;
            } else {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = p.vertices;

        // don't worry: the old one is still being pointed to
        cur_node = nullptr;
        p.num_traversable = 0;
        for (int j = 0; j < n; j++) {
            // Get neighbouring index of polygon
            int polygon_index = triangle.idxNeighTriangles[j];
            if (polygon_index >= P) {
                cerr << "Got a polygon index of " << polygon_index << endl;
            }

            if (polygon_index != -1) {
                p.num_traversable++;
            }
            ListNodePtr new_node = makeNode(nullptr, polygon_index);

            if (j == 0) {
                cur_node = new_node;
                p.polygons = cur_node;
            } else {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = p.polygons;

        p.area = getArea(mergedMesh, p.vertices);
        assert(p.area > 0);
    }
    mergedMesh.obstacles.reserve(fade2DMesh.obstacles.size());
    // Save obstacles to merged mesh
    for (const auto &obstacle : fade2DMesh.obstacles) {
        std::vector<Point> polygon(obstacle.size());
        int countPoints = 0;
        for (auto &point : obstacle) {
            polygon[countPoints] = parsers::Point{point.x(), point.y()};
            countPoints++;
        }
        mergedMesh.obstacles.push_back(polygon);
    }
}


void parsers::MapParser::generateMergedMesh(parsers::MergedMesh &mergedMesh) {
    // Process merge mesh
    mergeDeadEnd(mergedMesh);
    smartMerge(mergedMesh, true);
    checkCorrect(mergedMesh);
}


void parsers::MapParser::convertMapToMergedMesh(const std::string &filename, parsers::MergedMesh &mergedMesh) {
    parsers::Fade2DMesh fade2DMesh;
    // Create fade2d struct from map
    convertMapToFade2DMesh(filename, fade2DMesh);

    // Print and draw fade 2d mesh structure
    // mesh_plotter::printFade2DMesh(fade2DMesh, std::cout);
    // mesh_plotter::drawFade2D(fade2DMesh);

    // Create merged mesh structure from fade2d mesh structure
    readMergedMeshFromFade2DMesh(fade2DMesh, mergedMesh);

    // Process merge mesh
    generateMergedMesh(mergedMesh);

    // Print and draw merged mesh
    // mesh_plotter::printMergedMesh(mergedMesh, std::cout);
    // mesh_plotter::drawMergedMesh(mergedMesh);
}


void parsers::MapParser::convertMapToGeomMesh(const std::string &filename, parsers::GeomMesh &geomMesh) {
    parsers::MergedMesh mergedMesh;
    convertMapToMergedMesh(filename, mergedMesh);
    convertMergedMeshToGeomMesh(mergedMesh, geomMesh);
}


void
parsers::MapParser::convertFade2DMeshToGeomMesh(const parsers::Fade2DMesh &fade2DMesh, parsers::GeomMesh &geomMesh) {
    geomMesh.formatVersion = fade2DMesh.formatVersion;

    geomMesh.vertices.resize(fade2DMesh.vertices.size());
    geomMesh.polygons.resize(fade2DMesh.triangles.size());

    // Convert vertices
    for (int i = 0; i < (int) fade2DMesh.vertices.size(); i++) {
        parsers::Fade2DVertex fade2DVertex = fade2DMesh.vertices[i];
        GEOM_FADE2D::Point2 vertex = fade2DVertex.point;
        // Save X
        double x, y;
        vertex.xy(x, y);
        GeomVertex geomVertex;
        geomVertex.point = geom::Point<double>{x, y};
        // Save size of neighbours
        geomVertex.idxNeighPolygons.resize(fade2DVertex.idxNeighVertices.size());

        // Save indexes to file
        int counter = 0;
        for (auto index : fade2DVertex.idxNeighVertices) {
            geomVertex.idxNeighPolygons[counter] = index;
            counter++;
        }
        geomMesh.vertices[i] = geomVertex;
    }

    // Convert polygons
    int counterPolygons = 0;
    for (const auto &triangle : fade2DMesh.triangles) {
        GeomPolygon geomPolygon;
        geomPolygon.polygon.resize(triangle.idxVertices.size());
        geomPolygon.idxVertices.resize(triangle.idxVertices.size());
        geomPolygon.idxNeighPolygons.resize(triangle.idxNeighTriangles.size());
        // Vertices
        int counter = 0;
        for (int idx : triangle.idxVertices) {
            // Save index of vertex in polygon
            geomPolygon.idxVertices[counter] = idx;
            geomPolygon.polygon[counter] = geomMesh.vertices[idx].point;
            counter++;
        }
        // Neighbouring triangles
        int counterNeigh = 0;
        for (int idx : triangle.idxNeighTriangles) {
            geomPolygon.idxNeighPolygons[counterNeigh] = idx;
            counterNeigh++;
        }
        geomMesh.polygons[counterPolygons] = geomPolygon;
        counterPolygons++;
    }

    // Convert fade2d mesh's obstacles to geom mesh's obstacles
    geomMesh.obstacles.resize(fade2DMesh.obstacles.size());
    int countObstacles = 0;
    for (const auto &obstacle: fade2DMesh.obstacles) {
        geom::Polygon<double> polygon(obstacle.size());
        int countPoints = 0;
        for (const auto &point: obstacle) {
            polygon[countPoints] = geom::Point<double>{point.x(), point.y()};
            countPoints++;
        }
        geomMesh.obstacles[countObstacles] = polygon;
        countObstacles++;
    }
}

void
parsers::MapParser::convertMergedMeshToGeomMesh(parsers::MergedMesh &mergedMesh, parsers::GeomMesh &geomMesh) {
    int final_v, final_p;

    std::vector<int> vertex_mapping;
    vertex_mapping.resize(mergedMesh.mesh_vertices.size());
    {
        // We need to create a mapping from old-vertex to new-vertex.
        int next_index = 0;
        for (int i = 0; i < (int) mergedMesh.mesh_vertices.size(); i++) {
            if (mergedMesh.mesh_vertices[i].num_polygons != 0) {
                vertex_mapping[i] = next_index;
                next_index++;
            } else {
                vertex_mapping[i] = INT_MAX;
            }
        }
        final_v = next_index;
    }

    std::vector<int> polygon_mapping;
    polygon_mapping.resize(mergedMesh.mesh_polygons.size());
    {
        // We need to create a mapping from old-vertex to new-vertex.
        int next_index = 0;
        for (int i = 0; i < (int) mergedMesh.mesh_polygons.size(); i++) {
            if (mergedMesh.mesh_polygons[i].num_vertices != 0) {
                polygon_mapping[i] = next_index;
                next_index++;
            } else {
                polygon_mapping[i] = INT_MAX;
            }
        }
        final_p = next_index;
    }

#define get_v(v) ((v) == -1 ? -1 : vertex_mapping[v]);
#define get_p(p) ((p) == -1 ? -1 : polygon_mapping[mergedMesh.polygon_unions.find(p)]);

    geomMesh.polygons.reserve(final_p);
    geomMesh.vertices.resize(final_v);
    geomMesh.formatVersion = 2;


    for (int i = 0; i < (int) mergedMesh.mesh_vertices.size(); i++) {
        parsers::Vertex &v = mergedMesh.mesh_vertices[i];
        parsers::GeomVertex geomVertex;
        geomVertex.point.x = v.p.x;
        geomVertex.point.y = v.p.y;

        if (v.num_polygons == 0) {
            geomMesh.vertices[i] = geomVertex;
            continue;
        }

        geomVertex.idxNeighPolygons.reserve(v.num_polygons);
        int idx = get_p(v.polygons->val);
        geomVertex.idxNeighPolygons.push_back(idx);
        {
            int count = 1;
            parsers::ListNodePtr cur_node = v.polygons->next;
            while (cur_node != v.polygons) {
                assert(count < v.num_polygons);
                idx = get_p(cur_node->val);
                geomVertex.idxNeighPolygons.push_back(idx);
                cur_node = cur_node->next;
                count++;
            }
            assert(count == v.num_polygons);
        }
        geomMesh.vertices[i] = geomVertex;
    }

    for (int i = 0; i < (int) mergedMesh.mesh_polygons.size(); i++) {
        parsers::Polygon &p = mergedMesh.mesh_polygons[i];
        parsers::GeomPolygon geomPolygon;

        if (p.num_vertices == 0) {
            continue;
        }
        geomPolygon.idxVertices.reserve(p.num_vertices);
        int idx = get_v(p.vertices->val);
        geomPolygon.idxVertices.push_back(idx);
        geomPolygon.polygon.push_back(geomMesh.vertices[idx].point);
        {
            parsers::ListNodePtr cur_node = p.vertices->next;
            while (cur_node != p.vertices) {
                idx = get_v(cur_node->val);
                geomPolygon.idxVertices.push_back(idx);
                geomPolygon.polygon.push_back(geomMesh.vertices[idx].point);
                cur_node = cur_node->next;
            }
        }

        int idxPolygon = get_p(p.polygons->val);
        geomPolygon.idxNeighPolygons.push_back(idxPolygon);
        {
            parsers::ListNodePtr cur_node = p.polygons->next;
            while (cur_node != p.polygons) {
                idxPolygon = get_p(cur_node->val);
                geomPolygon.idxNeighPolygons.push_back(idxPolygon);
                cur_node = cur_node->next;
            }
        }
        geomMesh.polygons.push_back(geomPolygon);
    }

#undef get_p
#undef get_v


    // Convert merged mesh's obstacles to geom mesh's obstacles
    geomMesh.obstacles.resize(mergedMesh.obstacles.size());
    int countObstacles = 0;
    for (const auto &obstacle: mergedMesh.obstacles) {
        geom::Polygon<double> polygon(obstacle.size());
        int countPoints = 0;
        for (const auto &point: obstacle) {
            polygon[countPoints] = geom::Point<double>{point.x, point.y};
            countPoints++;
        }
        geomMesh.obstacles[countObstacles] = polygon;
        countObstacles++;
    }
}

/**
 * Private classes
 */

double parsers::MapParser::getArea(const MergedMesh &mergedMesh, parsers::ListNodePtr vertices) {
    // first point x second point + second point x third point + ...
    double out = 0;

    ListNodePtr start_vertex = vertices;
    bool is_first = true;

    while (is_first || start_vertex != vertices) {
        is_first = false;
        out += mergedMesh.mesh_vertices[vertices->val].p *
               mergedMesh.mesh_vertices[vertices->next->val].p;
        vertices = vertices->next;
    }
    return out;
}

bool parsers::MapParser::cw(const parsers::Point &a, const parsers::Point &b, const parsers::Point &c) {
    return (b - a) * (c - b) < -1e-8;
}

// Can polygon x merge with the polygon adjacent to the edge
// (v->next, v->next->next)?
// (The reason for this is because we don't have back pointers, and we need
// to have the vertex before the edge starts).
// Assume that v and p are "aligned", that is, they have been offset by the
// same amount.
// This also means that the actual polygon used will be p->next->next.
// Also assume that x is a valid non-merged polygon.
bool parsers::MapParser::canMerge(int x, parsers::ListNodePtr v, parsers::ListNodePtr p, MergedMesh &mergedMesh) {
    if (mergedMesh.polygon_unions.find(x) != x) {
        return false;
    }
    const int merge_index = mergedMesh.polygon_unions.find(p->go(2)->val);
    if (merge_index == -1) {
        return false;
    }
    const Polygon &to_merge = mergedMesh.mesh_polygons[merge_index];
    if (to_merge.num_vertices == 0) {
        return false;
    }

    // Define (v->next, v->next->next).
    const int A = v->go(1)->val;
    const int B = v->go(2)->val;

    // We want to find (B, A) inside to_merge's vertices.
    // In fact, we want to find the one BEFORE B. We'll call this merge_end.
    // Assert that we have good data - that is, if B appears, A must be next.
    // Also, we can't iterate for more than to_merge.num_vertices.
    ListNodePtr merge_end_v = to_merge.vertices;
    ListNodePtr merge_end_p = to_merge.polygons;
    int counter;
    counter = 0;
    while (merge_end_v->next->val != B) {
        merge_end_v = merge_end_v->next;
        merge_end_p = merge_end_p->next;
        counter++;
        assert(counter <= to_merge.num_vertices);
    }
    // Ensure that A comes after B.
    assert(merge_end_v->go(2)->val == A);
    // Ensure that the neighbouring polygon is x.
    assert(mergedMesh.polygon_unions.find(merge_end_p->go(2)->val) == x);

    // The merge will change
    // (v, A, B) to (v, A, [3 after merge_end_v]) and
    // (A, B, [3 after v]) to (merge_end_v, B, [3 after v]).
    // If the new ones are clockwise, we must return false.
#define P(ptr) mergedMesh.mesh_vertices[(ptr)->val].p
    if (cw(P(v), P(v->go(1)), P(merge_end_v->go(3)))) {
        return false;
    }

    if (cw(P(merge_end_v), P(v->go(2)), P(v->go(3)))) {
        return false;
    }
#undef P
    return true;
}

void parsers::MapParser::merge(int x, parsers::ListNodePtr v, parsers::ListNodePtr p, MergedMesh &mergedMesh) {
    assert(canMerge(x, v, p, mergedMesh));
    // Note that because of the way we're merging,
    // the resulting polygon will NOT always have a valid ListNodePtr, so
    // we need to set it ourself.

    const int merge_index = mergedMesh.polygon_unions.find(p->go(2)->val);

    Polygon &to_merge = mergedMesh.mesh_polygons[mergedMesh.polygon_unions.find(merge_index)];

    const int A = v->go(1)->val;
    const int B = v->go(2)->val;

    ListNodePtr merge_end_v = to_merge.vertices;
    ListNodePtr merge_end_p = to_merge.polygons;
    while (merge_end_v->next->val != B) {
        merge_end_v = merge_end_v->next;
        merge_end_p = merge_end_p->next;
    }

    // Our A should point to the thing which their A is pointing to.
    // Their B should point to the thing which our B is pointing to.
    ListNodePtr our_A_v_ptr = v->go(1);
    ListNodePtr our_A_p_ptr = p->go(1);
    ListNodePtr our_B_v_ptr = v->go(2);
    ListNodePtr our_B_p_ptr = p->go(2);

    ListNodePtr their_A_v_ptr = merge_end_v->go(2);
    ListNodePtr their_A_p_ptr = merge_end_p->go(2);
    ListNodePtr their_B_v_ptr = merge_end_v->go(1);
    ListNodePtr their_B_p_ptr = merge_end_p->go(1);

    our_A_v_ptr->next = their_A_v_ptr->next;
    our_A_p_ptr->next = their_A_p_ptr->next;
    their_B_v_ptr->next = our_B_v_ptr->next;
    their_B_p_ptr->next = our_B_p_ptr->next;

    // Set the our lists just in case something goes bad.
    // That is: don't set it to our B.
    Polygon &merged = mergedMesh.mesh_polygons[x];
    merged.vertices = our_A_v_ptr;
    merged.polygons = our_A_p_ptr;


    // Merge the numbers.
    merged.num_vertices += to_merge.num_vertices - 2;
    merged.num_traversable += to_merge.num_traversable - 2;
    merged.area += to_merge.area;

    // "Delete" the old one.
    to_merge = {0, 0, 0.0, nullptr, nullptr};

    // We now need to delete these in A and B.
    // A will go like (merge_index, x)
    // B will go like (x, merge_index)
    // We need to set both to just x.
    {
        // For A.
        // Once we find something which points to merge_index, point it to the
        // one after.
        ListNodePtr A_polys = mergedMesh.mesh_vertices[A].polygons;
        while (mergedMesh.polygon_unions.find(A_polys->next->val) != merge_index) {
            A_polys = A_polys->next;
        }
        A_polys->next = A_polys->next->next;
        // Set A to be this just in case.
        mergedMesh.mesh_vertices[A].polygons = A_polys;
        mergedMesh.mesh_vertices[A].num_polygons--;
    }
    {
        // For B.
        // Once we find something which is x, point it to the
        // one after.
        ListNodePtr B_polys = mergedMesh.mesh_vertices[B].polygons;
        while (mergedMesh.polygon_unions.find(B_polys->val) != x) {
            B_polys = B_polys->next;
            // cerr << "maybe even " << merge_index << endl;
            // cerr << "we want " << x << "but we got" << B_polys->val << endl;
        }
        B_polys->next = B_polys->next->next;
        // Set B to be this just in case.
        mergedMesh.mesh_vertices[B].polygons = B_polys;
        mergedMesh.mesh_vertices[B].num_polygons--;
    }
    // Do the union-find merge.
    // THIS NEEDS TO BE LAST.
    mergedMesh.polygon_unions.merge(x, merge_index);
}

void parsers::MapParser::checkCorrect(parsers::MergedMesh &mergedMesh) {
    for (int i = 0; i < (int) mergedMesh.mesh_vertices.size(); i++) {
        Vertex &v = mergedMesh.mesh_vertices[i];
        if (v.num_polygons == 0) {
            continue;
        }

        int count = 1;
        ListNodePtr cur_node = v.polygons->next;
        while (cur_node != v.polygons) {
            assert(count < v.num_polygons);
            cur_node = cur_node->next;
            count++;
        }
        assert(count == v.num_polygons);
    }

    for (int i = 0; i < (int) mergedMesh.mesh_polygons.size(); i++) {
        Polygon &p = mergedMesh.mesh_polygons[i];
        if (mergedMesh.polygon_unions.find(i) != i || p.num_vertices == 0) {
            // Has been merged.
            continue;
        }

        {
#define P(ptr) mergedMesh.mesh_vertices[(ptr)->val].p
            int count = 1;

            assert(!cw(P(p.vertices), P(p.vertices->next),
                       P(p.vertices->next->next)));
            canMerge(i, p.vertices, p.polygons, mergedMesh);

            ListNodePtr cur_node_v = p.vertices->next;
            ListNodePtr cur_node_p = p.polygons->next;
            while (cur_node_v != p.vertices) {
                assert(count < p.num_vertices);
                assert(!cw(P(cur_node_v), P(cur_node_v->next),
                           P(cur_node_v->next->next)));
                canMerge(i, cur_node_v, cur_node_p, mergedMesh);

                cur_node_v = cur_node_v->next;
                cur_node_p = cur_node_p->next;
                count++;
            }
            assert(count == p.num_vertices);
#undef P
        }

        {
            int count = 1;
            ListNodePtr cur_node = p.polygons->next;
            while (cur_node != p.polygons) {
                assert(count < p.num_vertices);
                cur_node = cur_node->next;
                count++;
            }
            assert(count == p.num_vertices);
        }
    }
}

void parsers::MapParser::mergeDeadEnd(parsers::MergedMesh &mergedMesh) {
    bool merged = false;
    do {
        merged = false;
        for (int i = 0; i < (int) mergedMesh.mesh_polygons.size(); i++) {
            Polygon &p = mergedMesh.mesh_polygons[i];
            if (mergedMesh.polygon_unions.find(i) != i || p.num_vertices == 0) {
                // Has been merged.
                continue;
            }
            // We want dead ends here.
            if (p.num_traversable != 1) {
                continue;
            }

            // Remember that the polygon we merge with is polygons->go(2).

            {
                const int merge_index = mergedMesh.polygon_unions.find(
                        p.polygons->go(2)->val);
                if (merge_index != -1 &&
                    mergedMesh.mesh_polygons[merge_index].num_traversable <= 2 &&
                    canMerge(i, p.vertices, p.polygons, mergedMesh)) {
                    merge(i, p.vertices, p.polygons, mergedMesh);
                    merged = true;
                    continue;
                }
            }

            ListNodePtr cur_node_v = p.vertices->next;
            ListNodePtr cur_node_p = p.polygons->next;
            while (cur_node_v != p.vertices) {
                const int merge_index = mergedMesh.polygon_unions.find(
                        cur_node_p->go(2)->val);
                if (merge_index != -1 &&
                    mergedMesh.mesh_polygons[merge_index].num_traversable <= 2 &&
                    canMerge(i, cur_node_v, cur_node_p, mergedMesh)) {
                    merge(i, cur_node_v, cur_node_p, mergedMesh);
                    merged = true;
                    break;
                }

                cur_node_v = cur_node_v->next;
                cur_node_p = cur_node_p->next;
            }
        }
    } while (merged);
}

void parsers::MapParser::smartMerge(parsers::MergedMesh &mergedMesh, bool keepDeadEnds) {
    priority_queue<SearchNode> pq;
    // As we aren't going to do pq updates, here's a shoddy workaround.
    vector<double> best_merge(mergedMesh.mesh_polygons.size(), -1);

    // Pushes a polygon onto the pq as a node.
    // Also updates best_merge.
    auto push_polygon = [&](int i) {
        if (i == -1) {
            return;
        }
        Polygon &p = mergedMesh.mesh_polygons[i];
        if (p.num_vertices == 0) {
            // Has been merged.
            return;
        }

        if (keepDeadEnds && p.num_traversable == 1) {
            // It's a dead end and we don't want to merge it.
            return;
        }

        SearchNode this_node = {i, -1};

        ListNodePtr cur_node_v = p.vertices;
        ListNodePtr cur_node_p = p.polygons;
        bool first = true;
        while (first || cur_node_v != p.vertices) {
            first = false;
            const int merge_index = mergedMesh.polygon_unions.find(cur_node_p->go(2)->val);
            if (merge_index != -1 &&
                (!keepDeadEnds ||
                 mergedMesh.mesh_polygons[merge_index].num_traversable > 1) &&
                canMerge(i, cur_node_v, cur_node_p, mergedMesh)) {
                this_node.area = max(this_node.area,
                                     p.area + mergedMesh.mesh_polygons[merge_index].area);
            }

            cur_node_v = cur_node_v->next;
            cur_node_p = cur_node_p->next;
        }

        // Chuck it on the pq... if we found a valid merge.
        if (this_node.area != -1) {
            pq.push(this_node);
            best_merge[i] = this_node.area;
        } else {
            // We need to invalidate this if there isn't a valid merge.
            best_merge[i] = -1;
        }
    };

    for (int i = 0; i < (int) mergedMesh.mesh_polygons.size(); i++) {
        push_polygon(i);
    }


    while (!pq.empty()) {
        SearchNode node = pq.top();
        pq.pop();
        if (abs(node.area - best_merge[node.index]) > 1e-8) {
            // Not the right node.
            continue;
        }
        // We got an actual node!
        const Polygon &p = mergedMesh.mesh_polygons[node.index];
        // Do the merge.
        // NOW do the merge.
        // We need to find it again, but that should be fine.
        {
            ListNodePtr cur_node_v = p.vertices;
            ListNodePtr cur_node_p = p.polygons;
            bool first = true;
            bool found = false;
            while (first || cur_node_v != p.vertices) {
                first = false;
                const int merge_index = mergedMesh.polygon_unions.find(
                        cur_node_p->go(2)->val);
                if (merge_index != -1 &&
                    (!keepDeadEnds ||
                     mergedMesh.mesh_polygons[merge_index].num_traversable > 1) &&
                    abs((p.area + mergedMesh.mesh_polygons[merge_index].area)
                        - node.area) < 1e-8 &&
                    canMerge(node.index, cur_node_v, cur_node_p, mergedMesh)) {
                    // Wait - before that, we need to invalidate the thing
                    // we merge with.
                    best_merge[merge_index] = -1;
                    merge(node.index, cur_node_v, cur_node_p, mergedMesh);
                    found = true;
                    break;
                }

                cur_node_v = cur_node_v->next;
                cur_node_p = cur_node_p->next;
            }
            assert(found);
        }

        // Update THIS merge.
        push_polygon(node.index);
        // Update the polygons around this merge.

        ListNodePtr cur_node_p = p.polygons;
        bool first = true;
        while (first || cur_node_p != p.polygons) {
            first = false;
            push_polygon(cur_node_p->val);
            cur_node_p = cur_node_p->next;
        }
    }
}

parsers::ListNodePtr parsers::MapParser::makeNode(parsers::ListNodePtr next, int val) {
    ListNodePtr out = new ListNode{next, val};
    listMergedNodes.push_back(out);
    return out;
}

void parsers::MapParser::deleteNodes() {
    for (auto x : listMergedNodes) {
        delete x;
    }
}