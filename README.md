# EdgeVis, PEA-E, polyanya PEA & TEA
This project contains implementations for computing visibility regions.
EdgeVis, PEA and TEA computes visibility for points and PEA-E implements visibility for edges in polygonal mesh.

The polyanya part is not the original implementation, but most of it is directly derived from it.

## Folder Hierarchy
 - dependencies: Folder with Fade2D.so compiled library and used IronHarvest maps converted by J. Mikula to better format.
 - scripts: Folder with python script for testing all implemented algorithms
 - source: Folder with the executable source and all implemented (or used) libraries.
 - paper: contains latex source for thesis

See [README.md](source/libs/haraborBased/README.md) in source/libs/haraborBased folder for information about algorithms.


## Setup
In source folder make build folder (unless its already there) and use cmake and make there.
```commandline
mkdir build
cd build
cmake ../source
make
```
The resulting executable is 
```
build/source/EdgeVis_example
```
You will need to install cairo, opencv, ncurses and boost libraries if you don't have them.
The usage of ncurses library may be deprecated, so maybe try make without it.

## Replicate tests from the paper
After setup you can replicate our experiments with
```
cd scripts
./run_tests.sh
```
It will take few hours and the output will be 6 .tex files with tables containing the results.
For some reason it seems there is a race condition in EdgeVis preprocessing. I did not find the cause yet.
All test works for me in case I close all other applications.


## Usage - EdgeVis_example
### Example call in build folder
This example call will compute visibility for 1M queries in map mp_2p_01 of Iron Harvest dataset.
Flag `-r` forces use of robust orientation tests by R. J. Shewchuk.
```
./source/EdgeVis_example -m scene_mp_2p_01 -n 1000000 -r
```

### Example output
```
------------------------------------------------------------------
* This software uses the Fade2D library under a student license. 
  Commercial use requires a valid commercial license, please visit 
  http://www.geom.at/licensing 

  Geom Software, Bernhard Kornberger
  C++ Freelancer, bkorn@geom.at
  http://www.geom.at

  (Fade2D serial no: 5303992d 6e923f87)

Preparing meshes, initializing Edgevis, TriVis and PolyVis solvers.
Tringular mesh has: 3673 triangles
Polygonal mesh has: 1716 polygons
1000000 random points generated in: 0.350803 seconds.


Map: scene_mp_2p_01
Number of iterations: 1000000
Orientation will be computed with robust test by R. J. Shewchuk.


Edgevis
Preprocessing time for Edge Visibility was 0.728988 seconds.
Preprocessing time for OptimNodesV1 was 0.195309 seconds.
Preprocessing time for OptimNodesV2 was 0.0402267 seconds.
Preprocessing time for OptimNodesV3 was 0.0652021 seconds.
EdgeVis v1: 
Total computation time: 8.86975 seconds.
Mean computation time: 8.86975e-06 seconds/point.
Mean number of nodes that were checked: 291.529
Mean number of resulting visibility polygon nodes: 136.322
EdgeVis v2: 
Total computation time: 8.5531 seconds.
Mean computation time: 8.5531e-06 seconds/point.
Mean number of nodes that were checked: 271.643
Mean number of resulting visibility polygon nodes: 136.322
EdgeVis v3: 
Total computation time: 8.52461 seconds.
Mean computation time: 8.52461e-06 seconds/point.
Mean number of nodes that were checked: 176.408
Mean number of resulting visibility polygon nodes: 136.321

PolyVis on triangles
Total computation time: 16.0011 seconds.
Mean computation time: 1.60011e-05 seconds/point.
Mean number of edge expansions: 228.482
Mean maximal depth of expansion: 47.4448

PolyVis on polygons
Total computation time: 12.5729 seconds.
Mean computation time: 1.25729e-05 seconds/point.
Mean number of edge expansions: 99.9491
Mean maximal depth of expansion: 18.1658

```

### Measurement CLI
```
General options:
  -h [ --help ]                           Produce this help message. 
                                           (*) Overwrites options: all.
  -m [ --map-name ] arg (=undefined)      Map name.
  --map-ext arg (=.mesh)                  Map file extension.
  --map-dir arg (=../dependencies/IronHarvest/mesh-maps/iron-harvest)
                                          Map file directory.
  --map arg                               Full path to the map file. 
                                           (*) Overwrites options: map_name, map_ext, map_dir.
  -n [ --n-random-samples ] arg (=1)      How many random samples should be generated.
  --random-seed arg (=1339350993)         Seed for the random generator (generated by std::random_device{}() by default).
  --debug                                 Run in debug mode.
  --save                                  Log resulting polygons to files (cannot measure performance)..
  -r [ --robust  ]                        Turn on robust orientation test by Shewchuk.
  --machine                               Save output in format dedicated for further SW processing
```

