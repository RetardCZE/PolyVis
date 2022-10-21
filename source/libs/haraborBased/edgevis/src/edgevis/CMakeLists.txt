FILE(GLOB SRCFILES_HELPERS helpers/*.cpp)
FILE(GLOB SRCFILES_STRUCTS structs/*.cpp)
FILE(GLOB SRCFILES_SEARCH search/*.cpp search/*.cc)

set(EDGEVIS_SOURCE_FILES ${SRCFILES_HELPERS} ${SRCFILES_STRUCTS} ${SRCFILES_SEARCH}  ${SRCFILES_UTILS_FADEUTILS} ${SRCFILES_PARSERS})

add_library(EdgeVis
       	   ${EDGEVIS_SOURCE_FILES})

target_link_libraries(EdgeVis GeomMesh Predicates Triangle Clipper cairo)
target_compile_options(EdgeVis PUBLIC -std=c++17)
