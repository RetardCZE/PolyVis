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
    geom::Polygon<double> border = {
                                    {x_min, y_min},
                                    {x_min, y_max+1},
                                    {x_max+1, y_max+1},
                                    {x_max+1, y_min},
                                    };
    cgm::CairoGeomDrawer cgm_drawer(x_max+1, y_max+1, 1.0);

    /// Create and save to PDF file
    cgm_drawer.OpenPDF("simple_map.pdf");
    cgm_drawer.DrawPlane(cgm::kColorBlack);
    cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
    cgm_drawer.DrawPolygons(obstacles, cgm::kColorWhite);
    cgm_drawer.Close();

}

int main(int argc, const char *const *argv)
{
    MapVisualizer drawer("mesh.txt");
    drawer.draw();
	return 0;
}