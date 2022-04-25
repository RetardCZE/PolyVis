cmake_minimum_required(VERSION 3.0.0)

include_directories(include
                    include/fade2D)

add_subdirectory(src)

target_include_directories(PolyAnya PUBLIC include
                                           include/fade2D)