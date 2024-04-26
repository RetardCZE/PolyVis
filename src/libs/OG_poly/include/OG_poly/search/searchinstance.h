#pragma once
#include "OG_poly/structs/searchnode.h"
#include "OG_poly/structs/successor.h"
#include "OG_poly/structs/mesh.h"
#include "OG_poly/structs/point.h"
#include "OG_poly/helpers/cpool.h"
#include "OG_poly/helpers/timer.h"
#include "utils/simple_clock.h"
#include <queue>
#include <vector>
#include <ctime>
#include <random>

namespace OG_poly
{

template<typename T, typename Compare = std::greater<T> >
struct PointerComp
{
    bool operator()(const T* x,
                    const T* y) const
    {
        return Compare()(*x, *y);
    }
};

typedef Mesh* MeshPtr;

class SearchInstance
{
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    private:
        OGwarthog::mem::cpool* node_pool;
        MeshPtr mesh;
        Point start, goal;

        SearchNodePtr final_node;
        int end_polygon; // set by init_search
        pq open_list;

        // Best g value for a specific vertex.
        std::vector<double> root_g_values;
        // Contains the current search id if the root has been reached by
        // the search.
        std::vector<int> root_search_ids;  // also used for root-level pruning
        int search_id;

        OGwarthog::timer timer;

        // Pre-initialised variables to use in search().
        Successor* search_successors;
        SearchNode* search_nodes_to_push;

        void init()
        {
            verbose = false;
            search_successors = new Successor [mesh->max_poly_sides + 2];
            search_nodes_to_push = new SearchNode [mesh->max_poly_sides + 2];
            node_pool = new OGwarthog::mem::cpool(sizeof(SearchNode));
            init_root_pruning();
        }
        void init_root_pruning()
        {
            assert(mesh != nullptr);
            search_id = 0;
            size_t num_vertices = mesh->mesh_vertices.size();
            root_g_values.resize(num_vertices);
            root_search_ids.resize(num_vertices);
        }
        void init_search()
        {
            assert(node_pool);
            node_pool->reclaim();
            search_id++;
            open_list = pq();
            final_node = nullptr;
            nodes_generated = 0;
            nodes_pushed = 0;
            nodes_popped = 0;
            nodes_pruned_post_pop = 0;
            successor_calls = 0;
            set_end_polygon();
            gen_initial_nodes();
        }
        PointLocation get_point_location(Point p);
        void set_end_polygon();
        void gen_initial_nodes();
        int succ_to_node(
            SearchNodePtr parent, Successor* successors,
            int num_succ, SearchNode* nodes
        );
        void print_node(SearchNodePtr node, std::ostream& outfile);

    public:
        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int nodes_popped;           // Nodes popped off open
        int nodes_pruned_post_pop;  // Nodes we prune right after popping off
        int successor_calls;        // Times we call get_successors
        bool verbose;
        custom::utils::SimpleClock clock;

        SearchInstance(){}
        SearchInstance(MeshPtr m) : mesh(m) { init(); }
        SearchInstance(MeshPtr m, Point s, Point g) :
            mesh(m), start(s), goal(g) { init(); }
        SearchInstance(SearchInstance const &) = delete;
        void operator=(SearchInstance const &x) = delete;

        ~SearchInstance()
        {
            if (node_pool)
            {
                delete node_pool;
            }
            delete[] search_successors;
            delete[] search_nodes_to_push;
        }

        void set_start_goal(Point s, Point g)
        {
            start = s;
            goal = g;
            final_node = nullptr;
        }

        bool search();
        double get_cost()
        {
            if (final_node == nullptr)
            {
                return -1;
            }

            return final_node->f;
        }

        int successors2nodes(
                SearchNodePtr parent, Successor* successors,
                int num_succ, SearchNode* nodes
        );
        void expand_edge(SearchNodePtr, Point root, int level);
        std::vector<SearchNodePtr> getInitNodes(Point &start);
        std::vector<Point> vertices;
        std::vector<Point> get_point_visibility(Point position, double &T1, double &T2, double &T3);

        std::vector<Point> generate_points(int n);
        double get_search_micro()
        {
            return timer.elapsed_time_micro();
        }

        void get_path_points(std::vector<Point>& out);
        void print_search_nodes(std::ostream& outfile);

};

}
