#include "polyanya/search/polyviz.h"

namespace polyanya {
    void
    PolyVis::set_visual_mesh(const parsers::GeomMesh &gmesh) {
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
                n.x = n.x - mesh->min_x;
                n.y = n.y - mesh->min_y;
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

    void PolyVis::reset_visu(){
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        cgm_drawer.Close();
        cgm_drawer = cgm::CairoGeomDrawer(mesh->max_x - mesh->min_x, mesh->max_y - mesh->min_y, 20);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh->min_x;
                n.y = n.y - mesh->min_y;
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
    PolyVis::visualise_segment(Point A, Point B, int color, float opacity) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh->min_x;
        vertex.y = A.y - mesh->min_y;
        vertex2.x = B.x - mesh->min_x;
        vertex2.y = B.y - mesh->min_y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        return;

    }

    void
    PolyVis::visualise_long_segment(Point A, Point B, int color, float opacity) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh->min_x;
        vertex.y = A.y - mesh->min_y;
        vertex2.x = B.x - mesh->min_x;
        vertex2.y = B.y - mesh->min_y;
        vertex2 = vertex2 - vertex;
        vertex2.x *= 100;
        vertex2.y *= 100;
        vertex2 = vertex + vertex2;

        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        return;

    }

    void
    PolyVis::visualise_point(Point A, int color, bool draw) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh->min_x;
        vertex.y = A.y - mesh->min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color]);
        return;

    }

    void
    PolyVis::make_frame() {
        cgm_drawer.SaveToPng(foldername + "/" + std::to_string(frames++) + ".png");
        return;
    }

    void
    PolyVis::visualise_named_point(Point A, int color, std::string str){
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh->min_x;
        vertex.y = A.y - mesh->min_y;
        cgm_drawer.DrawPoint(vertex, 0.1, cgm::kColorRed, 1, str);
        return;

    }

    void
    PolyVis::draw_base(bool boundaries) {

        for(auto p : visu_polygons){
            cgm_drawer.DrawPolygon(p, cgm::kColorLightBlue, 0.5);
        }
        if(boundaries){
            this->visualise_long_segment(seeker,currL, 0, 1);
            this->visualise_long_segment(seeker, currR, 0, 1);
        }
        this->visualise_named_point(seeker, 0, "Q");
        return;
    }

    void
    PolyVis::visualise_polygon(std::vector<Point> &p, int color, bool draw, bool outline) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorYellow};
        cgm::RGB light_colors[4] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue, cgm::kColorLightYellow};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x - mesh->min_x;
            vertex.y = v.y - mesh->min_y;
            polygon.push_back(vertex);
        }
        visu_polygons.push_back(polygon);

        //cgm_drawer.DrawPoints(polygon, 0.15, colors[color], 0.5);
        if(outline) {
            cgm_drawer.DrawPolygon(polygon, cgm::kColorLightBlue, 0.5);
            for (int i = 0; i < polygon.size() - 1; i++) {
                cgm_drawer.DrawLine(polygon[i], polygon[i + 1], 0.1, colors[color], 0.3);
            }
            this->visualise_named_point(seeker, 0, "Q");
        }
        return;
    }
}