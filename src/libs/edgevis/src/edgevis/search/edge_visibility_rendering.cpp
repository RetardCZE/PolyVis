#include "edgevis/structs/mesh.h"
#include "geom/geom.h"
#include "geom/utils.h"

namespace edgevis {
    using namespace cv;

    void
    Mesh::set_visual_mesh(const parsers::GeomMesh &gmesh) {
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;

        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - this->min_x;
                n.y = n.y - this->min_y;
            }
        }
        double x_min, x_max, y_min, y_max;
        geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
        geom::Polygon<double> border = {
                {x_min, y_min},
                {x_min, y_max},
                {x_max, y_max},
                {x_max, y_min},
        };

        cgm_drawer.OpenImage();
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

    }

    void
    Mesh::reset_visu(){
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        cgm_drawer.Close();
        cgm_drawer = cgm::CairoGeomDrawer(this->max_x - this->min_x, this->max_y - this->min_y, 20);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - this->min_x;
                n.y = n.y - this->min_y;
            }
        }
        double x_min, x_max, y_min, y_max;
        geom::ComputeLimits(free, x_min, x_max, y_min, y_max);
        geom::Polygon<double> border = {
                {x_min, y_min},
                {x_min, y_max},
                {x_max, y_max},
                {x_max, y_min},
        };
        cgm_drawer.OpenImage();
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);
        cgm_drawer.SaveToPng("debug_visu.png");
    }

    void
    Mesh::visualise_segment(Point A, Point B, int color, float opacity) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - this->min_x;
        vertex.y = A.y - this->min_y;
        vertex2.x = B.x - this->min_x;
        vertex2.y = B.y - this->min_y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    Mesh::visualise_point(Point A, int color, bool draw) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - this->min_x;
        vertex.y = A.y - this->min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color]);
        if(draw)
            cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    Mesh::visualise_polygon(std::vector<Point> &p, int color, bool draw) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorYellow};
        cgm::RGB light_colors[4] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue, cgm::kColorLightYellow};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x - this->min_x;
            vertex.y = v.y - this->min_y;
            polygon.push_back(vertex);
        }

        cgm_drawer.DrawPolygon(polygon, light_colors[color], 0.5);
        //cgm_drawer.DrawPoints(polygon, 0.15, colors[color], 0.5);

        for( int i = 0; i < polygon.size() - 1; i++){
            cgm_drawer.DrawLine(polygon[i], polygon[i+1], 0.1, colors[color], 0.3);
        }
        if(draw)
            cgm_drawer.SaveToPng("debug_visu.png");
        return;
    }
}
