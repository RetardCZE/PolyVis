cmake_minimum_required(VERSION 3.0.2)
project(PolyVis)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-O3)

## Set up optimization -  profiler flags
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/x86_64-linux-gnu/pkgconfig")
set(POLYANYA_DIR "polyanya")

include_directories(
        ${GTKMM_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${POLYANYA_DIR}
        ${POLYANYA_DIR}/utils
        ${CMAKE_SOURCE_DIR}
        draw_lib
)

## Using threads
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_subdirectory(draw_lib)
add_subdirectory(polyanya)

add_executable(${PROJECT_NAME}
        main.cpp
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE
        MAP_DATA_DIR="${PROJECT_SOURCE_DIR}/draw_lib/data/maps"
        PRIVATE -DTRILIBRARY -DANSI_DECLARATORS -DNO_TIMER
        )

target_link_libraries(${PROJECT_NAME}
        polygon_clipping
        polygonal_map
        cairo
        stdc++fs
        polyanya
        ${GTKMM_LIBRARIES}
        ${Boost_LIBRARIES}
        stdc++fs)