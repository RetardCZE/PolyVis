# Save all source files from polyanya library

FILE(GLOB SRCFILES_HELPERS helpers/*.cpp)
FILE(GLOB SRCFILES_SEARCH search/*.cpp)
FILE(GLOB SRCFILES_STRUCTS structs/*.cpp search/*.cc)

set(POLYANYA_SOURCE_FILES ${SRCFILES_HELPERS} ${SRCFILES_SEARCH} ${SRCFILES_STRUCTS} search/rendering.cpp)

add_library(PolyAnya
        ${POLYANYA_SOURCE_FILES}
        )

target_link_libraries(PolyAnya GeomMesh  Predicates Triangle Clipper cairo)
target_compile_options(PolyAnya PUBLIC -std=c++17)
