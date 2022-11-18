cmake_minimum_required(VERSION 3.0.0)
project(trivis_examples)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_library(FADE2D_LIBRARY NAMES fade2d PATHS ../dependencies/)
# Check if fade2d library was found
if (NOT FADE2D_LIBRARY)
    MESSAGE(STATUS "WARNING: Library FADE2D not found! Running script '${CMAKE_SOURCE_DIR}/pathfinding/polyanya/utils/scripts/install_fade2d.sh' solve the problem.")
else ()
    MESSAGE(STATUS "Found FADE2D")
endif ()

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

message("CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

add_subdirectory(libs) # include trivis and polyanya
# include boost:
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

# set(WITH_IPP ON)
# set(WITH_QT OFF)
# set(WITH_OPENGL ON)
# set(FORCE_VTK ON)
# set(WITH_TBB ON)
# set(WITH_GDAL ON)
# set(WITH_XINE ON)
# set(BUILD_EXAMPLES ON)
# set(ENABLE_PRECOMPILED_HEADERS OFF)
# set(WITH_GTK_2_X ON)

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
        -DINPUT_MAPS_DIR="${PROJECT_SOURCE_DIR}/../../IronHarvest/mesh-maps/iron-harvest"
        -DINPUT_CONVERTED_MAPS_DIR="${PROJECT_SOURCE_DIR}/../../IronHarvest/converted-maps/maps"
        )

FILE(GLOB SRC src/*.cpp)
set(SOURCES ${SRC})

add_executable(EdgeVis_example
        edgevis.cpp
        ${SOURCES}
        ${${PROJECT_NAME}_SRC})

target_compile_definitions(EdgeVis_example PRIVATE ${${PROJECT_NAME}_COMPILE_DEFINITIONS})

target_link_libraries(EdgeVis_example
        cairo
        ${CURSES_LIBRARIES}
        ${OpenCV_LIBS}
        TriVis
        PolyAnya
        EdgeVis
        GeomMesh
        ${FADE2D_LIBRARY}
        ${Boost_LIBRARIES})

# add_executable(EdgeVis_arbitrary_edge
#         edgevis_arbitrary_edge.cpp
#         ${SOURCES}
#         ${${PROJECT_NAME}_SRC})
# target_compile_definitions(EdgeVis_arbitrary_edge PRIVATE ${${PROJECT_NAME}_COMPILE_DEFINITIONS})
# target_link_libraries(EdgeVis_arbitrary_edge
#         cairo
#         TriVis
#         PolyAnya
#         EdgeVis
#         GeomMesh
#         ${FADE2D_LIBRARY}
#         ${Boost_LIBRARIES})