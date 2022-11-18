#include "edgevis/search/edge_visibility.h"

namespace edgevis {
    using namespace cv;
    void EdgeVisibility::visualiseOnline(Point p){
        auto surface = cgm_drawer.GetSurfacePtr();
        double relative_x = (p.x - mesh.min_x) / (mesh.max_x - mesh.min_x);
        double relative_y = (p.y - mesh.min_y) / (mesh.max_y - mesh.min_y);
        double w = cairo_image_surface_get_width(surface);
        double h = cairo_image_surface_get_height(surface);

        cv::Mat m(h, w, CV_8UC4);
        memcpy(m.data, cairo_image_surface_get_data(surface), 4 * w * h);

        if(widthCV > w){
            std::cout << "Maximum zoom out reached" << std::endl;
            widthCV = w;
        }
        int heightCV = widthCV * int(h/w);
        cv::Range cols(int(w*relative_x) - widthCV/2, int(w*relative_x) + widthCV/2);
        cv::Range rows((h - int(h*relative_y)) - heightCV/2,(h - int(h*relative_y)) + heightCV/2);
        if(rows.start < 0) rows.start = 0;
        if(rows.end >= w) rows.end = w-1;
        if(cols.start < 0) cols.start = 0;
        if(cols.end >= h) cols.end = h-1;

        Mat rez = m(rows, cols);
        Mat dst;
        if(w > h)
            resize(rez, dst, Size(windowWidth, int(windowWidth * h / w)), 0, 0, INTER_CUBIC);
        if(h > w)
            resize(rez, dst, Size(int(windowHeight * w / h), windowHeight), 0, 0, INTER_CUBIC);

        imshow("Map", dst);
        return;
    }


    void EdgeVisibility::visualise_vertex_indexes() {
        cgm_drawer.Close();

        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        int i = 0;

        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 5);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
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
        cgm_drawer.OpenPDF("vertices.pdf");
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

        for(auto v : mesh.mesh_vertices){
            this->visualise_named_point(v.p, 2, std::to_string(i++));
        }


        this->reset_visu();
    }

    void
    EdgeVisibility::visualise_heatmap(std::vector<Point> &points, std::vector<double> &compT, double tMax, double tMin,
                                      std::string name) {
        cgm_drawer.Close();

        int resolution = 300;
        double width = mesh.max_x - mesh.min_x;
        double height = mesh.max_y - mesh.min_y;
        double widthIncrement = width / resolution;
        double heightIncrement = height / resolution;
        std::vector<std::vector<double>> timeCols;
        std::vector<std::vector<int>> countCols;
        std::vector<double> timeRows;
        std::vector<int> countRows;

        for(int row = 0; row < resolution; row++){
            timeRows.push_back(0.0);
            countRows.push_back(1);
        }
        for(int col = 0; col < resolution; col++){
            timeCols.push_back(timeRows);
            countCols.push_back(countRows);
        }

        geom::Polygons<double> free;
        geom::Points<double> vertices;
        int i = 0;
        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 20);
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
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
        cgm_drawer.OpenPDF(name);
        cgm_drawer.DrawPlane(cgm::kColorBlack);
        cgm_drawer.DrawPolygon(border, cgm::kColorBlack);
        cgm_drawer.DrawPolygons(free, cgm::kColorWhite);

        for(int i = 0; i < points.size(); i++){
            auto pos = points[i];
            pos.x = pos.x - mesh.min_x;
            pos.y = pos.y - mesh.min_y;
            // auto t = (compT[i] - tMin + 0.00001) / (tMax - tMin);
            auto t = compT[i];
            //std::cout << pos << std::endl;
            timeCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)] = timeCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)] + t;
            countCols[int(pos.x / widthIncrement)][int(pos.y / heightIncrement)]++;
        }
        double tMax2 = 0;
        double tMin2 = 1;
        for(int col = 0; col < resolution; col++){
            for(int row = 0; row < resolution; row++){
                double heat = ((timeCols[col][row] / countCols[col][row]) - tMin) / (tMax - tMin);
                timeCols[col][row] = heat;
                if(tMax2 < heat) tMax2 = heat;
                if(tMin2 > heat and heat > 0) tMin2 = heat;
            }
        }
        for(int col = 0; col < resolution; col++){
            for(int row = 0; row < resolution; row++){
                Point p = {col * widthIncrement, row*heightIncrement};
                double heat = (timeCols[col][row] - tMin2) / (tMax2 - tMin2);
                this->visualise_heat_point(p, heat, widthIncrement, heightIncrement);
            }
        }

        this->reset_visu();
    }


    void
    EdgeVisibility::set_visual_mesh(const parsers::GeomMesh &gmesh) {
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
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
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

    void EdgeVisibility::reset_visu(){
        this->gmesh = gmesh;
        geom::Polygons<double> free;
        geom::Points<double> vertices;
        cgm_drawer.Close();
        cgm_drawer = cgm::CairoGeomDrawer(mesh.max_x - mesh.min_x, mesh.max_y - mesh.min_y, 5);
        for(auto v : this->gmesh.vertices){
            vertices.push_back(v.point);
        }
        for(auto p : this->gmesh.polygons){
            free.push_back(p.polygon);
        }
        for(auto &p : free){
            for(auto &n : p){
                n.x = n.x - mesh.min_x;
                n.y = n.y - mesh.min_y;
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
    EdgeVisibility::visualise_segment(Point A, Point B, int color, float opacity) {
        cgm::RGB colors[3] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        vertex2.x = B.x - mesh.min_x;
        vertex2.y = B.y - mesh.min_y;
        cgm_drawer.DrawLine(vertex, vertex2, 0.1, colors[color], 0.5);
        cgm_drawer.DrawPoint(vertex, 0.1, colors[color]);
        cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_point(Point A, int color, bool draw) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color]);
        if(draw)
            cgm_drawer.SaveToPng("debug_visu.png");
        return;

    }

    void
    EdgeVisibility::visualise_heat_point(Point A, double heat, double heightIncrement, double widthIncrement) {
        geom::Point<double> vertex;
        geom::Polygon<double> polygon;
        if(heat <= 0) return;

        int b = std::max(0, int(255*(1 - 2*heat)));
        int r = std::max(0, int(255*(2*heat - 1)));
        int g = 255 - b -r;

        vertex.x = A.x;
        vertex.y = A.y;
        polygon.push_back(vertex);

        vertex.x = A.x;
        vertex.y = A.y + heightIncrement;
        polygon.push_back(vertex);

        vertex.x = A.x + widthIncrement;
        vertex.y = A.y + heightIncrement;
        polygon.push_back(vertex);

        vertex.x = A.x + widthIncrement;
        vertex.y = A.y;
        polygon.push_back(vertex);



        cgm_drawer.DrawPolygon(polygon, cgm::RGB({r,g,b}), 0.5);

        return;

    }

    void
    EdgeVisibility::visualise_named_point(Point A, int color, std::string str){
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorBeige};
        geom::Point<double> vertex, vertex2;

        vertex.x = A.x - mesh.min_x;
        vertex.y = A.y - mesh.min_y;
        cgm_drawer.DrawPoint(vertex, 0.2, colors[color], 0.5, str);
        return;

    }

    void
    EdgeVisibility::visualise_polygon(std::vector<Point> &p, int color, bool draw) {
        cgm::RGB colors[4] = {cgm::kColorRed, cgm::kColorGreen, cgm::kColorBlue, cgm::kColorYellow};
        cgm::RGB light_colors[4] = {cgm::kColorLightPink, cgm::kColorLightGreen, cgm::kColorLightBlue, cgm::kColorLightYellow};
        geom::Polygon<double> polygon;
        geom::Point<double> vertex;

        for(auto v : p){
            vertex.x = v.x - mesh.min_x;
            vertex.y = v.y - mesh.min_y;
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