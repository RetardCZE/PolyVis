project(polygonal_map)
add_library(polygonal_map polygonal_map.cc)
target_include_directories(polygonal_map PUBLIC draw)
