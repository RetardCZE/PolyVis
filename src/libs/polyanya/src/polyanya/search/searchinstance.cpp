#include "polyanya/search/searchinstance.h"
#include "polyanya/search/expansion.h"
#include "polyanya/helpers/geometry.h"
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/vertex.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/point.h"
#include "polyanya/structs/consts.h"
#include <queue>
#include <vector>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <ctime>

namespace polyanya
{

PointLocation SearchInstance::get_point_location(Point p)
{
    assert(mesh != nullptr);
    PointLocation out = mesh->get_point_location(p);
    if (out.type == PointLocation::ON_CORNER_VERTEX_AMBIG)
    {
        // Add a few EPSILONS to the point and try again.
        static const Point CORRECTOR = {EPSILON * 10, EPSILON * 10};
        Point corrected = p + CORRECTOR;
        PointLocation corrected_loc = mesh->get_point_location(corrected);

        #ifndef NDEBUG
        if (verbose)
        {
            std::cerr << p << " " << corrected_loc << std::endl;
        }
        #endif

        switch (corrected_loc.type)
        {
            case PointLocation::ON_CORNER_VERTEX_AMBIG:
            case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
            case PointLocation::ON_NON_CORNER_VERTEX:
                #ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "Warning: corrected " << p << " lies on vertex"
                              << std::endl;
                }
                #endif
            case PointLocation::NOT_ON_MESH:
                #ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "Warning: completely ambiguous point at " << p
                              << std::endl;
                }
                #endif
                break;

            case PointLocation::IN_POLYGON:
            case PointLocation::ON_MESH_BORDER:
            // Note that ON_EDGE should be fine: any polygon works and there's
            // no need to special case successor generation.
            case PointLocation::ON_EDGE:
                out.poly1 = corrected_loc.poly1;
                break;

            default:
                // Should be impossible to reach.
                assert(false);
                break;
        }
    }
    return out;
}

int SearchInstance::successors2nodes(
    SearchNodePtr parent, Successor* successors, int num_succ,
    SearchNode* nodes
)
{
    // CUSTOM
    assert(mesh != nullptr);
    const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
    const std::vector<int>& V = polygon.vertices;
    const std::vector<int>& P = polygon.polygons;

    int out = 0;
    for (int i = 0; i < num_succ; i++)
    {
        const Successor& succ = successors[i];
        const int next_polygon = P[succ.poly_left_ind];
        const int left_vertex  = V[succ.poly_left_ind];
        const int right_vertex = succ.poly_left_ind ?
                                 V[succ.poly_left_ind - 1] :
                                 V.back();

        nodes[out++] = {nullptr, parent->root, succ.left, succ.right, left_vertex,
                        right_vertex, next_polygon, 0, 0};
    }
    return out;
}

std::vector<SearchNodePtr> SearchInstance::getInitNodes(Point &start)
{
    // CUSTOm
    std::vector<SearchNodePtr> list;
    const PointLocation pl = get_point_location(start);
    #define get_lazy(next, left, right) new (node_pool->allocate()) SearchNode \
        {nullptr, -1, start, start, left, right, next, 0, 0}

    #define v(vertex) mesh->mesh_vertices[vertex]
    const auto push_lazy = [&](SearchNodePtr lazy)
    {
        const int poly = lazy->next_polygon;
        if (poly == -1)
        {
            return;
        }

        // iterate over poly, throwing away vertices if needed
        const std::vector<int>& vertices =
            mesh->mesh_polygons[poly].vertices;

        Successor* successors = new Successor [vertices.size()];
        int last_vertex = vertices.back();
        int num_succ = 0;
        for (int i = 0; i < (int) vertices.size(); i++)
        {
            const int vertex = vertices[i];
            if (vertex == lazy->right_vertex ||
                last_vertex == lazy->left_vertex)
            {
                last_vertex = vertex;
                continue;
            }
            successors[num_succ++] =
                {Successor::OBSERVABLE, v(vertex).p,
                 v(last_vertex).p, i};
            last_vertex = vertex;
        }
        SearchNode* nodes = new SearchNode [num_succ];
        // needs to remove some pruning
        const int num_nodes = successors2nodes(lazy, successors,
                                               num_succ, nodes);
        delete[] successors;

        for (int i = 0; i < num_nodes; i++)
        {
            SearchNodePtr n = new (node_pool->allocate())
                SearchNode(nodes[i]);
            const Point& n_root = (n->root == -1 ? start :
                                   mesh->mesh_vertices[n->root].p);
            n->parent = lazy;
            list.push_back(n);
        }
        delete[] nodes;
        nodes_generated += num_nodes;
        nodes_pushed += num_nodes;
    };
    switch (pl.type)
    {
        // Don't bother.
        case PointLocation::NOT_ON_MESH:
            break;

        // Generate all in an arbirary polygon.
        case PointLocation::ON_CORNER_VERTEX_AMBIG:
            // It's possible that it's -1!
            if (pl.poly1 == -1)
            {
                break;
            }
        case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
        {
            std::vector<SearchNodePtr> lazies;
            for(int p : this->mesh->mesh_vertices[pl.vertex1].polygons){
                lazies.push_back(get_lazy(p, pl.vertex1, pl.vertex1));
            }
            for (SearchNodePtr l : lazies){
                push_lazy(l);
                nodes_generated++;
                if (final_node)
                {
                    return list;
                }
            }
        }
            break;
        // Generate all in the polygon.
        case PointLocation::IN_POLYGON:
        case PointLocation::ON_MESH_BORDER:
        {
            SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1);
            push_lazy(lazy);
            nodes_generated++;
        }
            break;

        case PointLocation::ON_EDGE:
            // Generate all in both polygons except for the shared side.
        {
            SearchNodePtr lazy1 = get_lazy(pl.poly2, pl.vertex1, pl.vertex2);
            SearchNodePtr lazy2 = get_lazy(pl.poly1, pl.vertex2, pl.vertex1);
            push_lazy(lazy1);
            nodes_generated++;
            if (final_node)
            {
                return list;
            }
            push_lazy(lazy2);
            nodes_generated++;
        }
            break;


        case PointLocation::ON_NON_CORNER_VERTEX:
        {
            for (int& poly : v(pl.vertex1).polygons)
            {
                SearchNodePtr lazy = get_lazy(poly, pl.vertex1, pl.vertex1);
                push_lazy(lazy);
                nodes_generated++;
                if (final_node)
                {
                    return list;
                }
            }
        }
            break;


        default:
            assert(false);
            break;
    }
    #undef v
    #undef get_lazy
    return list;
}

#define root_to_point(root) ((root) == -1 ? start : mesh->mesh_vertices[root].p)

void SearchInstance::print_node(SearchNodePtr node, std::ostream& outfile)
{
    outfile << "root=" << root_to_point(node->root) << "; left=" << node->left
            << "; right=" << node->right << "; f=" << node->f << ", g="
            << node->g;
    /*
    outfile << "; col=" << [&]() -> std::string
            {
                switch (node->col_type)
                {
                    case SearchNode::NOT:
                        return "NOT";
                    case SearchNode::RIGHT:
                        return "RIGHT";
                    case SearchNode::LEFT:
                        return "LEFT";
                    case SearchNode::LAZY:
                        return "LAZY";
                    default:
                        return "";
                }
            }();
    */
}

void SearchInstance::print_search_nodes(std::ostream& outfile)
{
    if (final_node == nullptr)
    {
        return;
    }
    SearchNodePtr cur_node = final_node;
    while (cur_node != nullptr)
    {
        print_node(cur_node, outfile);
        outfile << std::endl;
        mesh->print_polygon(outfile, cur_node->next_polygon);
        outfile << std::endl;
        cur_node = cur_node->parent;
    }
}

#undef root_to_point

}
