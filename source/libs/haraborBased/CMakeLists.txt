include_directories(include)
add_subdirectory(geomMesh)
# dependent on geomMesh
add_subdirectory(polyanya)
add_subdirectory(edgevis)

target_include_directories(GeomMesh PUBLIC include)
target_include_directories(EdgeVis PUBLIC include)
target_include_directories(PolyAnya PUBLIC include)
