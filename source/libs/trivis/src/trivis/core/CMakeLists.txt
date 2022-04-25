add_subdirectory(libs)

add_library(TriVisCore
        bucketing/bucketing.cc
        geom/intersections.cc
        geom/poly_map.cc
        geom/robust_geometry.cc
        geom/visibility_region.cc
        triangulation/tri_mesh.cc
        triangulation/constrained_delaunay.cc
        utils/clipper_utils.cc
        tri_vis.cc
        )

target_link_libraries(TriVisCore Predicates Triangle Clipper)

target_compile_options(TriVisCore PUBLIC -std=c++17)
