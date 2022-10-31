/**
 * File:    cairo_geom_drawer.h
 *
 * Date:    18.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef CAIRO_GEOM_DRAWER_CAIRO_GEOM_DRAWER_H_
#define CAIRO_GEOM_DRAWER_CAIRO_GEOM_DRAWER_H_

#include "geom.h"

#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>

#include <memory>

#include "colors.h"

namespace cairo_geom_drawer {

    static constexpr int kDefaultSizeX = 1920;
    static constexpr int kDefaultSizeY = 1080;
    static constexpr double kDefaultRes = 1.0;

    class CairoGeomDrawer {

    public:

        explicit CairoGeomDrawer(const double &scene_size_x = kDefaultSizeX,
                                 const double &scene_size_y = kDefaultSizeY,
                                 const double &res = kDefaultRes)
                : res_(res),
                  scene_size_x_(scene_size_x),
                  scene_size_y_(scene_size_y),
                  pic_size_x_(scene_size_x_ * res_),
                  pic_size_y_(scene_size_y_ * res_),
                  pic_size_x_int_(static_cast<int>(std::floor(pic_size_x_))),
                  pic_size_y_int_(static_cast<int>(std::floor(pic_size_y_))),
                  surface_ptr_(nullptr) {}


        [[nodiscard]] bool is_open() const { return is_open_; }

        void OpenPDF(const std::string &pdf_file) {
            is_open_ = true;
            surface_ptr_ = cairo_pdf_surface_create(pdf_file.c_str(), pic_size_x_, pic_size_y_);
        }

        void OpenImage() {
            is_open_ = true;
            surface_ptr_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, pic_size_x_int_, pic_size_y_int_);
        }

        void SaveToPng(const std::string &png_file) const {
            cairo_surface_write_to_png(surface_ptr_, png_file.c_str());
        }

        void Close() {
            is_open_ = false;
            cairo_surface_flush(surface_ptr_);
            cairo_surface_destroy(surface_ptr_);
        }

        void DrawPlane(const RGB &fill_color = kColorWhite) const {
            auto fill_color_01 = RGB01(fill_color);
            auto *cr = cairo_create(surface_ptr_);
            cairo_move_to(cr, 0, 0);
            cairo_line_to(cr, pic_size_x_, 0);
            cairo_line_to(cr, pic_size_x_, pic_size_y_);
            cairo_line_to(cr, 0, pic_size_y_);
            cairo_close_path(cr);
            cairo_set_source_rgb(cr, fill_color_01.r, fill_color_01.g, fill_color_01.b);
            cairo_fill(cr);
            cairo_destroy(cr);
        }

        template<typename T>
        void DrawPoint(const geom::Point<T> &p,
                       const T &radius = T(1),
                       const RGB &fill_color = kColorBlack,
                       const double &opacity = 1.0,
                       const std::string &text = std::string()) const {
            auto fill_color_01 = RGB01(fill_color);
            auto *cr = cairo_create(surface_ptr_);
            cairo_arc(cr, p.x * res_, pic_size_y_ - p.y * res_, radius * res_, 0.0, 2 * M_PI);
            cairo_set_source_rgba(cr, fill_color_01.r, fill_color_01.g, fill_color_01.b, opacity);
            cairo_set_font_size(cr, 10.0);
            // TODO valgrind warning - leak_definitely_lost
            cairo_show_text(cr, text.c_str());
            cairo_fill(cr);
            cairo_stroke(cr);
            cairo_destroy(cr);
        }

        template<typename T>
        void DrawPoints(const geom::Points<T> &points,
                        const T &radius = T(1),
                        const RGB &fill_color = kColorBlack,
                        const double &opacity = 1.0) const {
            for (auto &p : points) DrawPoint(p, radius, fill_color, opacity);
        }

        template<typename T>
        void DrawLine(const geom::Point<T> &p1,
                      const geom::Point<T> &p2,
                      const T &line_width = T(1),
                      const RGB &line_color = kColorBlack,
                      const double &opacity = 1.0,
                      const std::string &text = std::string()) const {
            auto line_color_01 = RGB01(line_color);
            auto *cr = cairo_create(surface_ptr_);
            cairo_move_to(cr, p1.x * res_, pic_size_y_ - p1.y * res_);
            cairo_line_to(cr, p2.x * res_, pic_size_y_ - p2.y * res_);
            cairo_set_line_width(cr, line_width * res_);
            cairo_set_source_rgba(cr, line_color_01.r, line_color_01.g, line_color_01.b, opacity);
            cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
            cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
            cairo_set_font_size(cr, 2.0);
            // TODO valgrind warning - leak_definitely_lost
//            cairo_show_text(cr, text.c_str());
            cairo_stroke(cr);
            cairo_destroy(cr);
        }

        template<typename T>
        void DrawPath(const geom::Points<T> &path,
                      const T &line_width = T(1),
                      const RGB &line_color = kColorBlack,
                      const double &opacity = 1.0) const {
            for (int i = 0; i < path.size() - 1; ++i) DrawLine(path[i], path[i + 1], line_width, line_color, opacity);
        }

        template<typename T>
        void DrawArc(const geom::Point<T> &p,
                     const T &radius = T(1),
                     const double &angle1 = 0.0,
                     const double &angle2 = M_PI,
                     const T &line_width = T(1),
                     const RGB &line_color = kColorBlack,
                     const double &opacity = 1.0) const {
            auto line_color_01 = RGB01(line_color);
            auto *cr = cairo_create(surface_ptr_);
            cairo_arc(cr, p.x * res_, pic_size_y_ - p.y * res_, radius * res_, angle1, angle2);
            cairo_set_line_width(cr, line_width * res_);
            cairo_set_source_rgba(cr, line_color_01.r, line_color_01.g, line_color_01.b, opacity);
            cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
            cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
            cairo_stroke(cr);
            cairo_destroy(cr);
        }

        template<typename T>
        void DrawPolygon(const geom::Polygon<T> &poly,
                         const RGB &fill_color = kColorBlack,
                         const double &opacity = 1.0) const {
            DrawPolygon(poly, true, fill_color, false, T(0), kColorBlack, opacity);
        }

        template<typename T>
        void DrawPolygon(const geom::Polygon<T> &poly,
                         const T &line_width,
                         const RGB &line_color = kColorBlack,
                         const double &opacity = 1.0) const {
            DrawPolygon(poly, false, kColorBlack, true, line_width, line_color, opacity);
        }

        template<typename T>
        void DrawPolygon(const geom::Polygon<T> &poly,
                         const T &line_width,
                         const RGB &line_color,
                         const RGB &fill_color,
                         const double &opacity = 1.0) const {
            DrawPolygon(poly, true, fill_color, true, line_width, line_color, opacity);
        }

        template<typename T>
        void DrawPolygons(const geom::Polygons<T> &poly,
                          const RGB &fill_color = kColorBlack,
                          const double &opacity = 1.0) const {
            DrawPolygons(poly, true, fill_color, false, T(0), kColorBlack, opacity);
        }

        template<typename T>
        void DrawPolygons(const geom::Polygons<T> &poly,
                          const T &line_width,
                          const RGB &line_color = kColorBlack,
                          const double &opacity = 1.0) const {
            DrawPolygons(poly, false, kColorBlack, true, line_width, line_color, opacity);
        }

        template<typename T>
        void DrawPolygons(const geom::Polygons<T> &poly,
                          const T &line_width,
                          const RGB &line_color,
                          const RGB &fill_color,
                          const double &opacity = 1.0) const {
            DrawPolygons(poly, true, fill_color, true, line_width, line_color, opacity);
        }

    protected:

        struct RGB01 {
            double r = 1.0;
            double g = 1.0;
            double b = 1.0;

            inline explicit RGB01(const RGB &rgb)
                    : r(static_cast<double>(rgb.r) / 255.0),
                      g(static_cast<double>(rgb.g) / 255.0),
                      b(static_cast<double>(rgb.b) / 255.0) {}
        };

    private:
        bool is_open_ = false;
        double res_;
        double scene_size_x_;
        double scene_size_y_;
        double pic_size_x_;
        double pic_size_y_;
        int pic_size_x_int_;
        int pic_size_y_int_;
        cairo_surface_t *surface_ptr_;

        template<typename T>
        void DrawPolygon(const geom::Polygon<T> &poly,
                         bool fill,
                         const RGB &fill_color,
                         bool stroke,
                         const T &line_width,
                         const RGB &line_color,
                         const double &opacity = 1.0) const {
            if (!poly.empty() && (fill || stroke)) {
                auto line_color_01 = RGB01(line_color);
                auto fill_color_01 = RGB01(fill_color);
                auto *cr = cairo_create(surface_ptr_);
                cairo_move_to(cr, poly[0].x * res_, pic_size_y_ - poly[0].y * res_);
                for (long unsigned int i = 1; i < poly.size(); ++i)
                    cairo_line_to(cr, (poly[i].x) * res_, pic_size_y_ - poly[i].y * res_);
                cairo_close_path(cr);
                if (fill) {
                    cairo_set_source_rgba(cr, fill_color_01.r, fill_color_01.g, fill_color_01.b, opacity);
                    cairo_fill_preserve(cr);
                }
                if (stroke) {
                    cairo_set_line_width(cr, line_width * res_);
                    cairo_set_source_rgba(cr, line_color_01.r, line_color_01.g, line_color_01.b, opacity);
                    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
                    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
                    cairo_stroke(cr);
                }
                cairo_destroy(cr);
            }
        }

        template<typename T>
        void DrawPolygons(const geom::Polygons<T> &polys,
                          bool fill,
                          const RGB &fill_color,
                          bool stroke,
                          const T &line_width,
                          const RGB &line_color,
                          const double &opacity = 1.0) const {
            for (auto &poly : polys) DrawPolygon(poly, fill, fill_color, stroke, line_width, line_color, opacity);
        }

    };

}

#endif //CAIRO_GEOM_DRAWER_CAIRO_GEOM_DRAWER_H_
