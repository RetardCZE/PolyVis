# Save all source files from polyanya library

FILE(GLOB SRCFILES_HELPERS helpers/*.cpp)
FILE(GLOB SRCFILES_STRUCTS structs/*.cpp)
FILE(GLOB SRCFILES_SEARCH search/*.cpp)
FILE(GLOB SRCFILES_UTILS_FADEUTILS fadeutils/*.cpp)
FILE(GLOB SRCFILES_PARSERS parsers/*.cpp)

set(POLYANYA_SOURCE_FILES ${SRCFILES_HELPERS} ${SRCFILES_STRUCTS} ${SRCFILES_SEARCH}  ${SRCFILES_UTILS_FADEUTILS} ${SRCFILES_PARSERS})

add_library(EdgeVis
        ${POLYANYA_SOURCE_FILES}
        )

target_link_libraries(EdgeVis ${FADE2D_LIBRARY} PolyAnya)

target_compile_options(EdgeVis PUBLIC -std=c++17)