#include "visualization.h"

MapVisualizer::MapVisualizer(parsers::GeomMesh &mesh){
    geom::Point<double> vertex;
    geom::Polygon<double> polygon;
    for(auto v : mesh.vertices){
        this->vertices.push_back(v.point);
    }
    for(auto p : mesh.polygons){
        this->free.push_back(p.polygon);
    }
}

MapVisualizer::~MapVisualizer(){
}

void
MapVisualizer::redraw(){
    /// Create draw_lib
    double x_min, x_max, y_min, y_max;
    geom::ComputeLimits(this->free, x_min, x_max, y_min, y_max);
    geom::Polygon<double> border = {
                                    {x_min, y_min},
                                    {x_min, y_max},
                                    {x_max, y_max},
                                    {x_max, y_min},
                                    };
    cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);

    /// Create and save to PDF file
    cgm_drawer.OpenPDF("simple_map.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
    cgm_drawer.DrawPolygons(this->free, cgm::kColorWhite);
    cgm_drawer.DrawPoints(this->visibility, 0.2, cgm::kColorGreen);
    cgm_drawer.DrawPolygon(this->visibility, cgm::kColorLightGreen);
    cgm_drawer.DrawPoint(this->seeker, 0.2, cgm::kColorRed);
    cgm_drawer.Close();
    cgm_drawer.OpenPDF("simple_map2.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
    cgm_drawer.DrawPolygons(this->free, cgm::kColorWhite);
    cgm_drawer.DrawPoint(this->seeker, 0.5, cgm::kColorRed);
    cgm_drawer.Close();
}

void MapVisualizer::set_visible_polygon(polyanya::Point p, std::vector<polyanya::Point> polygon){
    this->seeker.y = p.y;
    this->seeker.x = p.x;
    this->visibility.clear();
    for(auto v: polygon){
        geom::Point<double> vertex;
        vertex.x = v.x;
        vertex.y = v.y;
        this->visibility.push_back(vertex);
    }
}
