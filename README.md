# PolyVis
Repository for calculating polygon of visibility using polygons.

## Status
 
 - Currently playing with drawing some meshes generated from polyanya. 

## TODOs

 - Find how far can one get with polyanya successors.

## Setup
Git is currently not well managed, but for current functionality can be already tested.

In source folder make build folder (unless its already there) and use cmake and make there.
```commandline
cd source
mkdir build
cd build
cmake ..
make
```
Currently only compiled example is visualization of mesh from polyanya.
```commandline
./PolyVis
```
Output will be shown in build folder in simple_map.pdf
