# Save all source files from polyanya library

FILE(GLOB SRCFILES_HELPERS helpers/*.cpp)
FILE(GLOB SRCFILES_SEARCH search/*.cpp)
FILE(GLOB SRCFILES_STRUCTS structs/*.cpp search/*.cc)

set(OGPOLYANYA_SOURCE_FILES ${SRCFILES_HELPERS} ${SRCFILES_SEARCH} ${SRCFILES_STRUCTS})

add_library(OGPolyAnya
        ${OGPOLYANYA_SOURCE_FILES}
        )

target_link_libraries(OGPolyAnya GeomMesh  Predicates Triangle Clipper cairo)
target_compile_options(OGPolyAnya PUBLIC -std=c++17)
