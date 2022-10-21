#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/structs/point.h"
#include <vector>

namespace polyanya
{

// Gets the h value of a search node with interval l-r and root "root",
// given a goal.
double get_h_value(const Point& root, Point goal,
                   const Point& l, const Point& r);

// Generates the successors of the search node and sets them in the successor
// vector. Returns number of successors generated.
int get_successors(SearchNode& node, const Point& start, const Mesh& mesh,
                   Successor* successors);

int get_successors2(SearchNode& node, const Point& start, const Mesh& mesh,
                   Successor* successors);
}
