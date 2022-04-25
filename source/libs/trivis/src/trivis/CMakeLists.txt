add_subdirectory(core)

add_library(TriVis
        map_coverage/macs.cc
        map_coverage/map_coverage.cc
        map_coverage/random_points.cc
        map_coverage/vis_region_approx.cc
        )

target_link_libraries(TriVis TriVisCore)

target_compile_options(TriVis PUBLIC -std=c++17)
