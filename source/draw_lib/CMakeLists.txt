cmake_minimum_required(VERSION 3.0.2)
project(draw_lib)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-O3)

add_subdirectory(polygon_clipping)
add_subdirectory(polygonal_map)
include_directories(PUBLIC geom)


add_executable(${PROJECT_NAME}_example
        example.cc
        )

target_compile_definitions(${PROJECT_NAME}_example PRIVATE
        MAP_DATA_DIR="${PROJECT_SOURCE_DIR}/data/maps"
        PRIVATE -DTRILIBRARY -DANSI_DECLARATORS -DNO_TIMER
        )

target_link_libraries(${PROJECT_NAME}_example
        polygon_clipping
        polygonal_map
        cairo
        stdc++fs)
