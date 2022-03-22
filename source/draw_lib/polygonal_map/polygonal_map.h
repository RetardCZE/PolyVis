/**
 * File:    polygonal_map.h
 *
 * Date:    19.08.2020
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef POLYGONAL_MAP_POLYGONAL_MAP_H_
#define POLYGONAL_MAP_POLYGONAL_MAP_H_

#include <utility>

#include "draw/map_drawer.h"

namespace polygonal_map {

std::ostream &operator<<(std::ostream &os, const geom::Polygon<double> &polygon);

std::ostream &operator<<(std::ostream &os, const geom::Polygons<double> &polygons);

struct Limits {
  double x_min = 0.0;
  double x_max = 1.0;
  double y_min = 0.0;
  double y_max = 1.0;
};

class PolygonalMap {

 public:

  enum class LoadCode : int {
    kOK = 0,
    kNotOpenedOrFound = 1,
    kInvalid = 2
  };

  enum class ValidityCode : int {
    kOK = 0,
    kEmpty = 1,
    kNoBorder = 2,
    kMoreBorders = 3,
    kEmptyBorder = 4,
    kEmptyObstacle = 5,
    kNotPolygonBorder = 6,
    kNotPolygonObstacle = 7,
    kObstacleOutsideBorder = 8,
    kNonEmptyObstaclePairIntersection = 9
  };

  explicit PolygonalMap() = default;

  explicit PolygonalMap(geom::Polygons<double> polygons) : map_(std::move(polygons)) {}

  [[nodiscard]] const auto &get() const { return map_; }

  void set_polygons(const geom::Polygons<double> &polygons) { map_ = polygons; }

  [[nodiscard]] static ValidityCode Valid(const geom::Polygons<double> &polygons);

  [[nodiscard]] ValidityCode Valid() const { return Valid(map_); }

  static void Fix(geom::Polygons<double> &polygons);

  void Fix() { return Fix(map_); }

  [[nodiscard]] static bool Load(const std::string &file, geom::Polygons<double> &polygons) noexcept(false);

  [[nodiscard]] bool Load(const std::string &file) noexcept(false) { return Load(file, map_); }

  [[nodiscard]] static LoadCode LoadSafely(const std::string &file, geom::Polygons<double> &polygons);

  [[nodiscard]] LoadCode LoadSafely(const std::string &file) { return LoadSafely(file, map_); }

  static void Shift(const double &shift_x, const double &shift_y, geom::Polygons<double> &polygons);

  void Shift(const double &shift_x = 0.0, const double &shift_y = 0.0) { Shift(shift_x, shift_y, map_); }

  static void Round(const double &rounder, geom::Polygons<double> &polygons);

  void Round(const double &rounder = 1.0) { Round(rounder, map_); }

  [[nodiscard]] static Limits ComputeLimits(const geom::Polygons<double> &polygons);

  [[nodiscard]] Limits ComputeLimits() { return ComputeLimits(map_); }

  static void Standardize(const double &shift_factor, const double &rounder, geom::Polygons<double> &polygons);

  void Standardize(const double &shift_factor = 0.0, const double &rounder = 1.0) {
    Standardize(shift_factor, rounder, map_);
  }

  [[nodiscard]] static draw::MapDrawer CreateMapDrawer(const geom::Polygons<double> &polygons);

  [[nodiscard]] draw::MapDrawer CreateMapDrawer() const { return CreateMapDrawer(map_); }

  [[nodiscard]] static std::string ToString(const geom::Polygon<double> &polygon,
                                            bool parenthesis = true,
                                            const std::string &delimiter_1 = ", ",
                                            const std::string &delimiter_2 = "\n");

  [[nodiscard]] static std::string ToString(const geom::Polygons<double> &polygons,
                                            bool parenthesis = true,
                                            const std::string &delimiter_1 = ", ",
                                            const std::string &delimiter_2 = "\n",
                                            const std::string &delimiter_3 = "\n\n");

  [[nodiscard]] std::string ToString(bool parenthesis = true,
                                     const std::string &delimiter_1 = ", ",
                                     const std::string &delimiter_2 = "\n",
                                     const std::string &delimiter_3 = "\n\n") {
    return ToString(map_, parenthesis, delimiter_1, delimiter_2, delimiter_3);
  }

  friend std::ostream &operator<<(std::ostream &os, const PolygonalMap &map) { return os << map.map_; }

 private:

  geom::Polygons<double> map_;

};

}

#endif //POLYGONAL_MAP_POLYGONAL_MAP_H_
