#include "visualization.h"

MapVisualizer::MapVisualizer(char* file){
    this->my_file = std::ifstream(file, std::ios::binary);
}

MapVisualizer::~MapVisualizer(){
    this->my_file.close();
    }

void
MapVisualizer::parse_mesh(){
    std::stringstream buffer;
    buffer << this->my_file.rdbuf();

    int line = 0;
    int polygon_count, vertex_count, v;
    geom::Point<double> vertex;
    geom::Polygon<double> polygon;

    std::string tp;
    std::vector<std::string> numbers;
    std::string delimiter = " ";


    while(std::getline(buffer, tp)){
        numbers.clear();
        line++;
        size_t pos = 0;
        while ((pos = tp.find(delimiter)) != std::string::npos) {
            numbers.push_back(tp.substr(0, pos));
            tp.erase(0, pos + delimiter.length());
        }
        numbers.push_back(tp.substr(0, pos));
        if(line == 3){
            polygon_count = std::stoi(numbers[1]);
            vertex_count = std::stoi(numbers[0]);
        }else if(line <= vertex_count + 3 && line > 3){
            vertex = {(double)std::stoi(numbers[1]), (double)std::stoi(numbers[0])};
            this->vertices.push_back(vertex);
        }else if(line > 3){
            polygon.clear();
            v = std::stoi(numbers[0]);
            for(int i = 1; i <= v; i++){
                polygon.push_back(this->vertices[std::stoi(numbers[i])]);
            }
            this->free.push_back(polygon);
        }
    }
    std::cout << this->vertices.size() << std::endl;
    std::cout << this->free.size() << std::endl;
}

void
MapVisualizer::draw(){
    this->parse_mesh();
    geom::Polygons<double> obstacles = this->free;

    /// Create draw_lib
    double x_min, x_max, y_min, y_max;
    geom::ComputeLimits(obstacles, x_min, x_max, y_min, y_max);
    cgm::CairoGeomDrawer cgm_drawer(x_max, y_max, 1.0);

    /// Create and save to PDF file
    cgm_drawer.OpenPDF("simple_map.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    //cgm_drawer.DrawPolygon(border, cgm::kColorWhite);
    cgm_drawer.DrawPolygons(obstacles, cgm::kColorBlack);
    cgm_drawer.Close();

}

int main(int argc, const char *const *argv)
{
    MapVisualizer drawer("mesh.txt");
    drawer.draw();
	/// Basics of geometry.
/*
    /// Polygon clipping.
    geom::Polygon<double> poly1 = {{16, 4},
                                   {18, 7},
                                   {18, 9},
                                   {15, 7}};
    geom::Polygon<double> poly2 = {{17, 4},
                                   {17, 8},
                                   {15, 6},
                                   {15, 3}};
    cgm_drawer.OpenPDF("poly_clipping.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorWhite);
    cgm_drawer.DrawPolygons(obstacles, cgm::kColorBlack);
    clip::CPolygons solution;
    clip::Clip(clip::CClipType::ctIntersection, clip::Geom2Clipper(poly1), clip::Geom2Clipper(poly2), solution);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(solution), cgm::kColorOrange);
    clip::Clip(clip::CClipType::ctDifference, clip::Geom2Clipper(poly1), clip::Geom2Clipper(poly2), solution);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(solution), cgm::kColorRed, 0.5);
    cgm_drawer.DrawPolygon(poly1, 0.1, cgm::kColorLimeGreen);
    cgm_drawer.DrawPolygon(poly2, 0.1, cgm::kColorDeepSkyBlue);
    cgm_drawer.DrawPoint({17, 4}, 0.5, cgm::kColorRed, 0.5);
    std::cout << "Orientations: ";
    for (const auto &p : simple_map) std::cout << clip::Orientation(clip::Geom2Clipper(p)) << " ";
    std::cout << std::endl;
    std::cout << "Relative area: " << clip::Area(clip::Geom2Clipper(simple_map)) << std::endl;
    auto simple_map_clip_offset = clip::Geom2Clipper(simple_map);
    clip::Offset(simple_map_clip_offset, -0.5);
    cgm_drawer.DrawPolygons(clip::Clipper2Geom<double>(simple_map_clip_offset), 0.1, cgm::kColorSilver);
    cgm_drawer.Close();

    /// Load the map safely.
    map::PolygonalMap poly_map;
    std::string file_name = std::string(MAP_DATA_DIR) + "/jari-huge/jari-huge.txt";
    //if (poly_map.LoadSafely(file_name) == map::PolygonalMap::LoadCode::kOK) {
    //    // do sth
    //} else {
    //    // error
    //}
    auto load_code = poly_map.LoadSafely(file_name);
    switch (load_code) {
        case map::PolygonalMap::LoadCode::kOK: {
            std::cout << "OK: File " << file_name << " loaded." << std::endl;
            break;
        }
        case map::PolygonalMap::LoadCode::kNotOpenedOrFound: {
            std::cout << "ERROR: File " << file_name << " could not opened or found!" << std::endl;
            return 1;
        }
        case map::PolygonalMap::LoadCode::kInvalid: {
            std::cout << "ERROR: File " << file_name << " is invalid!" << std::endl;
            return 1;
        }
    }

    /// Standardize the map (important for good functionality).
    poly_map.Standardize(0.02, 1e6);

    /// Check if valid.
    std::cout << "Validity: " << static_cast<int>(poly_map.Valid()) << std::endl;

    /// Draw the map.
    // poly_map.set_polygons(simple_map);
    auto drawer = poly_map.CreateMapDrawer();
    drawer.OpenPDF("map.pdf");
    drawer.DrawPlane(draw::kColorDeepSkyBlue);
    drawer.DrawBorders(draw::kColorOrange);
    drawer.DrawObstacles(draw::kColorLimeGreen);
    // drawer.DrawMap();
    drawer.Close();
    */
	return 0;
}