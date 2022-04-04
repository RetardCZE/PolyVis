# PolyVis
Repository for calculating polygon of visibility using polygons.

## Status
 
 - Polyanya can be used to find visibility polygon with successors related to root.
 - Algorithm stops at some polygons on mesh for some reason - need to be investigated.
   Maybe badly implemented recursion.

## TODOs

 - Find why polyanya stops at some point. Successors are find well I think, but are not
   Transformed to nodes to continue search - WHY?

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
