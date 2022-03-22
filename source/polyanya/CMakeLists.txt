# Save all source files from polyanya library
project(polyanya)
add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-O3)

FILE(GLOB SRCFILES_HELPERS helpers/*.cpp)
FILE(GLOB SRCFILES_SEARCH search/*.cpp)
FILE(GLOB SRCFILES_STRUCTS structs/*.cpp)
FILE(GLOB SRCFILES_UTILS_FADEUTILS utils/fadeutils/*.cpp)

set(POLYANYA_SOURCE_FILES ${SRCFILES_HELPERS} ${SRCFILES_SEARCH} ${SRCFILES_STRUCTS} ${SRCFILES_UTILS_FADEUTILS} ${SRCFILES_PLOTTERS})
include_directories( utils/fade2d )
add_library(polyanya ${POLYANYA_SOURCE_FILES})