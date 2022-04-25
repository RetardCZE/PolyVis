# PolyVis
Repository for calculating polygon of visibility using polygons.

## Status
 
 - Polyanya can be used to find visibility polygon, but takes more than TriVis.

## TODOs

 - Measure time for current Polyanya and TriVis. Apply known optimization for polyanya.
 - Implement computation of polygon of visibility from line.

## Setup
Git is currently not well managed, but for current functionality can be already tested.

In source folder make build folder (unless its already there) and use cmake and make there.
```commandline
mkdir build
cd build
cmake ../source
make
```
Currently only compiled example is visualization of mesh from polyanya.
```commandline
./PolyVis
```
Output will be shown in build folder in simple_map.pdf
