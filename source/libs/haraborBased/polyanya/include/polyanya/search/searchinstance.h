#pragma once
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/point.h"
#include "polyanya/helpers/cpool.h"
#include "polyanya/helpers/timer.h"
#include <queue>
#include <vector>
#include <ctime>

namespace polyanya
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
    private:
        warthog::mem::cpool* node_pool;
        MeshPtr mesh;
        Point start, goal;

        SearchNodePtr final_node;

        warthog::timer timer;


        void init()
        {
            verbose = false;
            node_pool = new warthog::mem::cpool(sizeof(SearchNode));
        }

        PointLocation get_point_location(Point p);

        void print_node(SearchNodePtr node, std::ostream& outfile);

    public:
        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int successor_calls;        // Times we call get_successors
        bool verbose;

        std::vector<SearchNodePtr> getInitNodes(Point &start);
        int successors2nodes(
            SearchNodePtr parent, Successor* successors,
            int num_succ, SearchNode* nodes
        );


        SearchInstance() { }
        SearchInstance(MeshPtr m) : mesh(m) { init(); }
        SearchInstance(SearchInstance const &) = delete;
        void operator=(SearchInstance const &x) = delete;
        ~SearchInstance()
        {
            if (node_pool)
            {
                delete node_pool;
            }
        }

        void print_search_nodes(std::ostream& outfile);

};

}
