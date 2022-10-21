FILE(GLOB SRCFILES_UTILS_FADEUTILS fadeutils/*.cpp)
FILE(GLOB SRCFILES_PARSERS parsers/*.cpp)

set(GEOMMESH_SOURCE_FILES ${SRCFILES_UTILS_FADEUTILS} ${SRCFILES_PARSERS})

add_library(GeomMesh
            ${GEOMMESH_SOURCE_FILES})

target_link_libraries(GeomMesh ${FADE2D_LIBRARY} cairo)
target_compile_options(GeomMesh PUBLIC -std=c++17)
