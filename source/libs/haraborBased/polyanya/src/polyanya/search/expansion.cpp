#include "polyanya/search/expansion.h"
#include "polyanya/structs/searchnode.h"
#include "polyanya/structs/successor.h"
#include "polyanya/structs/mesh.h"
#include "polyanya/helpers/geometry.h"
#include "polyanya/structs/vertex.h"
#include "polyanya/structs/point.h"
#include "polyanya/structs/consts.h"
#include <cassert>
#include <vector>

namespace polyanya
{
// Internal binary search helper.
// All indices must be within the range [0, 2 * N - 1] to make binary search
// easier. You can normalise an index with this macro:
#define normalise(index) (index) - ((index) >= N ? N : 0)
// Assume that there exists at least one element within the range which
// satisifies the predicate.
template<typename Type, typename Pred>
inline int binary_search(const std::vector<int>& arr, const int N,
                         const std::vector<Type>& objects, int lower, int upper,
                         const Pred pred, const bool is_upper_bound)
{
    if (lower == upper) return lower;
    int best_so_far = -1;
    while (lower <= upper)
    {
        const int mid = lower + (upper - lower) / 2;
        const bool matches_pred = pred(objects[arr[normalise(mid)]]);
        if (matches_pred)
        {
            best_so_far = mid;
        }
        // If we're looking for an upper bound:
            // If we match the predicate, go higher.
            // If not, go lower.
        // If we're looking for a lower bound:
            // If we match the predicate, go lower.
            // If not, go higher.
        if (matches_pred == is_upper_bound)
        {
            // Either "upper bound AND matches pred"
            // or "lower bound AND doesn't match pred"
            // We should go higher, so increase the lower bound.
            lower = mid + 1;
        }
        else
        {
            // The opposite.
            // Decrease the upper bound.
            upper = mid - 1;
        }
    }
    return best_so_far;
}

int expand(SearchNode& node, const Point& start, const Mesh& mesh,
           Successor* successors)
{
    // If the next polygon is -1 we dont have any successors (we shouldnt even look for them)
    if(node.next_polygon == -1) return 0;

    const Polygon& polygon = mesh.mesh_polygons[node.next_polygon];
    const std::vector<Vertex>& mesh_vertices = mesh.mesh_vertices;
    // V, P and N are solely used for conciseness
    const std::vector<int>& V = polygon.vertices;
    const int N = (int) V.size();

    const Point& root = (node.root == -1 ? start : mesh_vertices[node.root].p);

    int out = 0;

    if (N == 3)
    {
        int p1; // V[p1] = t2. Used for poly_left_ind for 1-2 successors.
        int p2; // V[p2] = t3. Used for poly_left_ind for 2-3 successors.
        // Note that p3 is redundant, as that's the polygon we came from.

        // The right point of the triangle.
        const Point& t1 = mesh_vertices[node.right_vertex].p;
        // The middle point of the triangle.
        const Point& t2 = [&]() -> const Point&
        {
            // horrible hacky lambda which also sets p1/p2

            // Let's get p1, p2 and t2.
            if (V[0] == node.right_vertex)
            {
                // t1 = V[0], t2 = V[1], t3 = V[2]
                p1 = 1;
                p2 = 2;
                return mesh_vertices[V[1]].p;
            }
            else if (V[0] == node.left_vertex)
            {
                // t1 = V[1], t2 = V[2], t3 = V[0]
                p1 = 2;
                p2 = 0;
                return mesh_vertices[V[2]].p;
            }
            else
            {
                // t1 = V[2], t2 = V[0], t3 = V[1]
                p1 = 0;
                p2 = 1;
                return mesh_vertices[V[0]].p;
            }
        }();
        // The left point of the triangle.
        const Point& t3 = mesh_vertices[node.left_vertex].p;



        const Point& L = node.left;
        const Point& R = node.right;

        // Now we need to check the orientation of root-L-t2.
        // TODO: precompute a shared term for getting orientation,
        // like t2 - root.
        switch (get_orientation(root, L, t2))
        {
            case Orientation::CCW:
            {
                // LI in (1, 2)
                // RI in [1, 2)

                // TODO: precompute shared constants (assuming the compiler
                // doesn't)
                const Point LI = line_intersect(t1, t2, root, L);
                const Point RI = (R == t1 ? t1 :
                                  line_intersect(t1, t2, root, R));

                // observable(RI, LI)
                successors[0] = {
                    Successor::OBSERVABLE,
                    LI, RI,
                    p1 // a 1-2 successor
                };


                return 1;
            }

            case Orientation::COLLINEAR:
            {
                // LI = 2
                // RI in [1, 2)
                const Point RI = (R == t1 ? t1 :
                                  line_intersect(t1, t2, root, R));

                // observable(RI, 2)
                successors[0] = {
                    Successor::OBSERVABLE,
                    t2, RI,
                    p1 // a 1-2 successor
                };


                return 1;
            }

            case Orientation::CW:
            {
                // LI in (2, 3]
                const Point LI = (L == t3 ? t3 :
                                  line_intersect(t2, t3, root, L));

                // Now we need to check the orientation of root-R-t2.
                switch (get_orientation(root, R, t2))
                {
                    case Orientation::CW:
                    {
                        // RI in (2, 3)
                        const Point RI = line_intersect(t2, t3, root, R);

                        // observable(RI, LI)
                        successors[0] = {
                            Successor::OBSERVABLE,
                            LI, RI,
                            p2 // a 2-3 successor
                        };

                        return 1;
                    }

                    case Orientation::COLLINEAR:
                    {
                        // RI = 2
                        // if we can turn right

                        // observable(2, LI)
                        successors[0] = {
                            Successor::OBSERVABLE,
                            LI, t2,
                            p2 // a 2-3 successor
                        };

                        return 1;
                    }

                    case Orientation::CCW:
                    {
                        // RI in [1, 2)
                        const Point RI = (R == t1 ? t1 :
                                          line_intersect(t1, t2, root, R));

                        // observable(RI, 2)
                        successors[0] = {
                            Successor::OBSERVABLE,
                            t2, RI,
                            p1 // a 1-2 successor
                        };

                        // observable(2, LI)
                        successors[1] = {
                            Successor::OBSERVABLE,
                            LI, t2,
                            p2 // a 2-3 successor
                        };

                        return 2;
                    }

                    default:
                        assert(false);
                }
            }

            default:
                assert(false);
        }
    }


    // It is not collinear.
    // Find the starting vertex (the "right" vertex).

    // Note that "_ind" means "index in V/P",
    // "_vertex" means "index of mesh_vertices".
    // "_vertex_obj" means "object of the vertex" and
    // "_p" means "point".
    const int right_ind = [&]() -> int
    {
        // TODO: Compare to std::find.
        int temp = 0; // position of vertex in V
        while (V[temp] != node.right_vertex)
        {
            temp++;
            assert(temp < N);
        }
        return temp;
    }();
    // Note that left_ind MUST be greater than right_ind.
    // This will make binary searching easier.
    const int left_ind = N + right_ind - 1;

    assert(V[normalise(left_ind)] == node.left_vertex);

    // Find whether we can turn at either endpoint.
    const Vertex& right_vertex_obj = mesh_vertices[node.right_vertex];
    const Vertex& left_vertex_obj  = mesh_vertices[V[normalise(left_ind)]];

    const Point& right_p = right_vertex_obj.p;
    const Point& left_p  = left_vertex_obj.p;
    const bool right_lies_vertex = right_p == node.right;
    const bool left_lies_vertex  = left_p == node.left;

    // Macro for getting a point from a polygon point index.
    #define index2point(index) mesh_vertices[V[index]].p

    // find the transition between non-observable-right and observable.
    // we will call this A, defined by:
    // "first P such that root-right-p is strictly CCW".
    // lower bound is right+1, as root-right-right is not CCW (it is collinear).
    // upper bound is left.
    // the "transition" will lie in the range [A-1, A)

    const Point root_right = node.right - root;
    const int A = [&]()
    {
        if (right_lies_vertex)
        {
            // Check whether root-right-right+1 is collinear or CCW.
            if (root_right *
                (index2point(normalise(right_ind + 1)) - node.right) >
                -EPSILON)
            {
                // Intersects at right, so...
                // we should use right_ind+1!
                return right_ind + 1;
            }
        }
        return binary_search(V, N, mesh_vertices, right_ind + 1, left_ind,
            [&root_right, &node](const Vertex& v)
            {
                // STRICTLY CCW.
                return root_right * (v.p - node.right) > EPSILON;
            }, false
        );
    }();
    assert(A != -1);
    const int normalised_A = normalise(A),
              normalised_Am1 = normalise(A-1);

    const Point& A_p = index2point(normalised_A);
    const Point& Am1_p = index2point(normalised_Am1);
    const Point right_intersect = right_lies_vertex && A == right_ind + 1 ? node.right : line_intersect(A_p, Am1_p, root, node.right);

    // find the transition between observable and non-observable-left.
    // we will call this B, defined by:
    // "first P such that root-left-p is strictly CW".
    // lower-bound is A - 1 (in the same segment as A).
    // upper bound is left-1, as we don't want root-left-left.
    // the "transition" will lie in the range (B, B+1]
    const Point root_left = node.left - root;
    const int B = [&]()
    {
        if (left_lies_vertex)
        {
            // Check whether root-left-left-1 is collinear or CW.
            if (root_left *
                (index2point(normalise(left_ind - 1)) - node.left) <
                EPSILON)
            {
                // Intersects at left, so...
                // we should use left_ind-1!
                return left_ind - 1;
            }
        }
        return binary_search(V, N, mesh_vertices, A - 1, left_ind - 1,
            [&root_left, &node](const Vertex& v)
            {
                // STRICTLY CW.
                return root_left * (v.p - node.left) < -EPSILON;
            }, true
        );
    }();
    assert(B != -1);
    const int normalised_B = normalise(B),
              normalised_Bp1 = normalise(B+1);
    const Point& B_p = index2point(normalised_B);
    const Point& Bp1_p = index2point(normalised_Bp1);
    const Point left_intersect = left_lies_vertex && B == left_ind - 1 ? node.left : line_intersect(B_p, Bp1_p, root, node.left);

    // Macro to update this_inde/last_ind.
    #define update_ind() last_ind = cur_ind++; if (cur_ind == N) cur_ind = 0

    // Start at Am1.
    // last_node = right_intersect
    // If index is normalised_Bp1, go from last_node to left_intersect.
    // (And terminate too!)
    // Else, go to the end and set that as last_node

    // Special case when there are NO observable successors.
    if (A == B + 2)
    {
        // Do nothing.
    }
    // Special case when there only exists one observable successor.
    // Note that we used the non-normalised indices for this.
    else if (A == B + 1)
    {
        successors[out++] = {
            Successor::OBSERVABLE,
            left_intersect, right_intersect,
            normalised_A // (the same as normalised_Bp1)
        };
    }
    else
    {
        // Generate first (probably non-maximal) successor
        // (right_intersect-A)
        successors[out++] = {
            Successor::OBSERVABLE,
            A_p, right_intersect,
            normalised_A
        };

        // Generate all guaranteed-maximal successors.
        // Should generate B-A of them.
        int last_ind = normalised_A;
        int cur_ind = normalise(A+1);

        #ifndef NDEBUG
        int counter = 0;
        #endif

        while (last_ind != normalised_B)
        {
            #ifndef NDEBUG
            counter++;
            #endif

            // Generate last-cur.
            successors[out++] = {
                Successor::OBSERVABLE,
                index2point(cur_ind), index2point(last_ind),
                cur_ind
            };

            update_ind();
        }

        #ifndef DEBUG
        assert(counter == B - A);
        #endif

        // Generate last (probably non-maximal) successor
        // (B-left_intersect)
        successors[out++] = {
            Successor::OBSERVABLE,
            left_intersect, B_p,
            normalised_Bp1
        };
    }

    #undef update_ind
    #undef index_to_point

    return out;
}
#undef normalise
}
