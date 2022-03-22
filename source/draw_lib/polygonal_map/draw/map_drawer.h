/**
 * File:    map_drawer.h
 *
 * Date:    19.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#include <utility>

#include "../../cairo_geom_drawer/cairo_geom_drawer.h"

#ifndef POLYGONAL_MAP_MAP_DRAWER_H_
#define POLYGONAL_MAP_MAP_DRAWER_H_

namespace polygonal_map::draw {

using namespace cairo_geom_drawer;

class MapDrawer : public CairoGeomDrawer {

 public:
  explicit MapDrawer(geom::Polygons<double> borders,
                     geom::Polygons<double> obstacles,
                     const double &scene_size_x = kDefaultSizeX,
                     const double &scene_size_y = kDefaultSizeY,
                     const double &res = kDefaultRes)
      : CairoGeomDrawer(scene_size_x, scene_size_y, res),
        borders_(std::move(borders)),
        obstacles_(std::move(obstacles)) {
  }

  void DrawBorders(const RGB &fill_color = kColorBlack, const double &opacity = 1.0) const {
    DrawPolygons(borders_, fill_color, opacity);
  }

  void DrawBorders(const double &line_width,
                          const RGB &line_color = kColorBlack,
                          const double &opacity = 1.0) const {
    DrawPolygons(borders_, line_width, line_color, opacity);
  }

  void DrawBorders(const double &line_width,
                          const RGB &line_color,
                          const RGB &fill_color,
                          const double &opacity = 1.0) const {
    DrawPolygons(borders_, line_width, line_color, fill_color, opacity);
  }

  void DrawObstacles(const RGB &fill_color = kColorBlack, const double &opacity = 1.0) const {
    DrawPolygons(obstacles_, fill_color, opacity);
  }

  void DrawObstacles(const double &line_width,
                            const RGB &line_color = kColorBlack,
                            const double &opacity = 1.0) const {
    DrawPolygons(obstacles_, line_width, line_color, opacity);
  }

  void DrawObstacles(const double &line_width,
                            const RGB &line_color,
                            const RGB &fill_color,
                            const double &opacity = 1.0) const {
    DrawPolygons(obstacles_, line_width, line_color, fill_color, opacity);
  }

  void DrawMap(const RGB &border_color = kColorWhite,
               const RGB &obstacle_color = kColorBlack) const {
    DrawPlane(obstacle_color);
    DrawBorders(border_color);
    DrawObstacles(obstacle_color);
  }

 private:
  geom::Polygons<double> borders_;
  geom::Polygons<double> obstacles_;

};

}

#endif //POLYGONAL_MAP_MAP_DRAWER_H_
