cmake_minimum_required(VERSION 3.0.0)
project(trivis_examples)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_library(FADE2D_LIBRARY NAMES fade2d PATHS /usr/local/lib/FADE2D/)
# Check if fade2d library was found
if (NOT FADE2D_LIBRARY)
    MESSAGE(STATUS "WARNING: Library FADE2D not found! Running script '${CMAKE_SOURCE_DIR}/pathfinding/polyanya/utils/scripts/install_fade2d.sh' solve the problem.")
else ()
    MESSAGE(STATUS "Found FADE2D")
endif ()

message("CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

add_subdirectory(libs) # include trivis and polyanya

# include boost:
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.65 REQUIRED COMPONENTS program_options system filesystem log_setup log)

include_directories(
        include
)

set(${PROJECT_NAME}_SRC
        src/data_loading/load_map.cc
        src/drawing/random_colors.cc
        src/drawing/drawing.cc
        src/logging/logging.cc
        )

set(${PROJECT_NAME}_COMPILE_DEFINITIONS
        -DINPUT_MAPS_DIR="${PROJECT_SOURCE_DIR}/data/maps"
        )

# add_executable(trivis_debug trivis_debug.cc ${${PROJECT_NAME}_SRC})
# target_compile_definitions(trivis_debug PRIVATE ${${PROJECT_NAME}_COMPILE_DEFINITIONS})
# target_link_libraries(trivis_debug TriVis ${Boost_LIBRARIES} cairo)
#
# add_executable(trivis_examples trivis_examples.cc ${${PROJECT_NAME}_SRC})
# target_compile_definitions(trivis_examples PRIVATE ${${PROJECT_NAME}_COMPILE_DEFINITIONS})
# target_link_libraries(trivis_examples TriVis ${Boost_LIBRARIES} cairo)

FILE(GLOB SRC src/*.cpp)
set(SOURCES ${SRC})

add_executable(EdgeVis_example
        edgevis.cpp
        ${SOURCES}
        ${${PROJECT_NAME}_SRC})

target_compile_definitions(EdgeVis_example PRIVATE ${${PROJECT_NAME}_COMPILE_DEFINITIONS})

target_link_libraries(EdgeVis_example
        TriVis
        cairo
        PolyAnya
        EdgeVis
        ${FADE2D_LIBRARY}
        ${Boost_LIBRARIES})