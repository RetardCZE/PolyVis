/**
 * File:    utils.h
 *
 * Date:   24.03.2021
 * Author:  Lukas Fanta
 * E-mail:  fantalukas2108@gmail.com
 *
 */

#ifndef CETSP_POLYANYA_PARSERS_UTILS_H
#define CETSP_POLYANYA_PARSERS_UTILS_H

/*
 * System includes
 */
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <numeric>
#include <climits>
#include <cmath>
#include <queue>

/*
 * Local includes
 */
#include "geomMesh/fadeutils/polymap.h"
#include "geom/geom.h"

namespace parsers {

    using namespace std;

    typedef vector<GEOM_FADE2D::Point2> Fade2DPolygon;

    // Struct of vertex using draw library
    struct GeomVertex {
        geom::Point<double> point;
        // Index of vertex and correspond list of neighbouring vertices/polygons
        vector<int> idxNeighPolygons;
    };

    // Struct of polygon using draw library
    struct GeomPolygon {
        // Polygon represented in draw library
        geom::Polygon<double> polygon;
        // Indexes of vertices for given triangle
        vector<int> idxVertices;
        // Indexes of neighbouring polygons
        vector<int> idxNeighPolygons;
    };

    // Structure describe mesh using draw library
    struct GeomMesh {
        int formatVersion = 2;
        vector<GeomVertex> vertices;
        vector<GeomPolygon> polygons;
        geom::Polygons<double> obstacles;
    };


    // Struct of triangle used in parsing process
    struct Fade2DTriangle {
        GEOM_FADE2D::Triangle2 *ptrTriangle;
        // Indexes of vertices for given triangle
        vector<int> idxVertices;
        // Indexes of neighbouring triangles
        vector<int> idxNeighTriangles;
    };

    // Struct of vertex used in parsing process
    struct Fade2DVertex {
        GEOM_FADE2D::Point2 point;
        // Index of vertex and correspond list of neighbours
        vector<int> idxNeighVertices;
    };

    struct Fade2DMesh {
        int formatVersion = 2;
        vector<Fade2DVertex> vertices;
        vector<Fade2DTriangle> triangles;
        vector<Fade2DPolygon> obstacles;
    };

    // We need union find!
    struct UnionFind {
        vector<int> parent;

        UnionFind() = default;

        explicit UnionFind(int n) : parent(n) {
            iota(parent.begin(), parent.end(), 0);
        }

        int find(int x);

        void merge(int x, int y);
    };

    // We need a circular linked list of sorts.
    // This is going to be used a lot - for polys around point and for merging the
    // two polygon arrays together.
    // We'll just use a std::shared_ptr to handle our memory...
    struct ListNode {
        ListNode *next;
        int val;

        ListNode *go(int n) const {
            ListNode *out = next;
            for (int i = 1; i < n; i++) {
                out = out->next;
            }
            return out;
        }
    };

    typedef ListNode *ListNodePtr;

    struct Point {
        double x, y;

        Point operator+(const Point &other) const {
            return {x + other.x, y + other.y};
        }

        Point operator-(const Point &other) const {
            return {x - other.x, y - other.y};
        }

        double operator*(const Point &other) const {
            return x * other.y - y * other.x;
        }
    };

    struct Vertex {
        Point p;
        int num_polygons;
        ListNodePtr polygons;
    };

    struct Polygon {
        int num_vertices;
        int num_traversable;
        double area;
        ListNodePtr vertices;
        // Stores the original polygons.
        // To get the actual polygon, do polygon_unions.find on the polygon you get.
        ListNodePtr polygons;
    };

    struct SearchNode {
        // Index of poly.
        int index;
        // Area of the best tentative merge.
        double area;

        // Comparison.
        // Always take the "biggest" search node in a priority queue.
        bool operator<(const SearchNode &other) const {
            return area < other.area;
        }

        bool operator>(const SearchNode &other) const {
            return area > other.area;
        }
    };

    struct MergedMesh {
        // We'll keep all vertices, but we may throw them out in the end if num_polygons
        // is 0.
        // We'll figure it out once we're finished.
        vector<Vertex> mesh_vertices;

        // We'll also keep all polygons, but we'll throw them out like above.
        vector<Polygon> mesh_polygons;

        UnionFind polygon_unions;
        // Polygons representing obstacles
        vector<std::vector<Point>> obstacles;
    };

    /**
     * Compute limits of draw mesh
     * @param map
     * @param xMin
     * @param yMin
     * @param xMax
     * @param yMax
     */
    void computeLimits(const GeomMesh &map, double &xMin, double &yMin, double &xMax, double &yMax);
}

#endif //CETSP_POLYANYA_PARSERS_UTILS_H
