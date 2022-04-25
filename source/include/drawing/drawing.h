/**
 * File:   drawing.h
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_EXAMPLES_DRAWING_DRAWING_H_
#define TRIVIS_EXAMPLES_DRAWING_DRAWING_H_

#include "drawing/cairo_geom_drawer.h"

#include "trivis/core/geom/generic_geom_utils.h"
#include "trivis/core/geom/poly_map.h"

namespace trivis_examples::drawing {

class MapDrawer : public CairoGeomDrawer {

public:

    explicit MapDrawer(
        trivis::core::geom::FPolygons borders,
        trivis::core::geom::FPolygons holes,
        const double scene_size_x = CairoGeomDrawer::kDefaultSizeX,
        const double scene_size_y = CairoGeomDrawer::kDefaultSizeY,
        const double res = CairoGeomDrawer::kDefaultRes,
        const double offset_x = 0.0,
        const double offset_y = 0.0
    )
        : CairoGeomDrawer(scene_size_x, scene_size_y, res, offset_x, offset_y),
          _borders(std::move(borders)),
          _holes(std::move(holes)) {
    }

    ~MapDrawer() = default;

    MapDrawer(const MapDrawer &other) = default;

    MapDrawer(MapDrawer &&other) noexcept = default;

    MapDrawer &operator=(const MapDrawer &other) = default;

    MapDrawer &operator=(MapDrawer &&other) noexcept = default;

    void DrawBorders(
        const RGB &fill_color = kColorBlack,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_borders, fill_color, opacity);
    }

    void DrawBorders(
        const double &line_width,
        const RGB &line_color = kColorBlack,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_borders, line_width, line_color, opacity);
    }

    void DrawBorders(
        const double &line_width,
        const RGB &line_color,
        const RGB &fill_color,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_borders, line_width, line_color, fill_color, opacity);
    }

    void DrawHoles(
        const RGB &fill_color = kColorBlack,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_holes, fill_color, opacity);
    }

    void DrawHoles(
        const double &line_width,
        const RGB &line_color = kColorBlack,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_holes, line_width, line_color, opacity);
    }

    void DrawHoles(
        const double &line_width,
        const RGB &line_color,
        const RGB &fill_color,
        const double &opacity = 1.0
    ) const {
        DrawPolygons(_holes, line_width, line_color, fill_color, opacity);
    }

    void DrawMap(
        const RGB &border_color = kColorWhite,
        const RGB &obstacle_color = kColorBlack
    ) const {
        DrawPlane(obstacle_color);
        DrawBorders(border_color);
        DrawHoles(obstacle_color);
    }

private:
    trivis::core::geom::FPolygons _borders;
    trivis::core::geom::FPolygons _holes;

};

static constexpr auto kDefaultMakeMapDrawerRes = .025;
static constexpr auto KDefaultMakeMapDrawerRelativeFrameWidth = .02;

MapDrawer MakeMapDrawer(
    const trivis::core::geom::FPolygons &borders,
    const trivis::core::geom::FPolygons &holes,
    double res = kDefaultMakeMapDrawerRes,
    double relative_frame_width = KDefaultMakeMapDrawerRelativeFrameWidth
);

inline MapDrawer MakeMapDrawer(
    const trivis::core::geom::FPolygon &border,
    const trivis::core::geom::FPolygons &holes,
    double res = kDefaultMakeMapDrawerRes,
    double relative_frame_width = KDefaultMakeMapDrawerRelativeFrameWidth
) {
    return MakeMapDrawer(trivis::core::geom::FPolygons(1, border), holes, res, relative_frame_width);
}

inline MapDrawer MakeMapDrawer(
    const trivis::core::geom::PolyMap &map,
    double res = kDefaultMakeMapDrawerRes,
    double relative_frame_width = KDefaultMakeMapDrawerRelativeFrameWidth
) {
    return MakeMapDrawer(map.border(), map.holes(), res, relative_frame_width);
}

}

#endif //TRIVIS_EXAMPLES_DRAWING_DRAWING_H_
