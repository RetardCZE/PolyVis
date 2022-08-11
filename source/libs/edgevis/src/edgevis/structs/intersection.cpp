#include "edgevis/structs/intersection.h"

using namespace edgevis;
namespace robust_geom{
    bool SegmentLineIntersection::calculate(Mesh &mesh) {
        Point A, B, C, D;
        A = mesh.mesh_vertices[this->a].p;
        B = mesh.mesh_vertices[this->b].p;
        C = mesh.mesh_vertices[this->c].p;
        D = mesh.mesh_vertices[this->d].p;

        int check = LineSegmentIntersectionGeneral(A, B, C, D, this->p);
        if(check == 0 || check == 5 || check == 6){
            is_calculated = true;
        }else{
            is_calculated = false;
        }
        return is_calculated;
    }
}
