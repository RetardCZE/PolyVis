# EdgeVis and visibility from segments

This folder contains implementation of PEA-E and EdgeVis algorithms.
robust_geometry.cc and intersections.cc were taken from TriVis project written by J.Mikula.

The basic structure was taken from polyanya so some definitions may be the same.

I've defined new structs:
 - edge
 - intersection
 - optimnode (V1,V2,V3)
 - searchpoint

and changed:
 - searchnode
 - mesh

The algorithms are not derived from polyanya at all and were implemented only by me.

Short description:
 - edge_visibility: Computation of visibility from edges.
 - edge_visibility_rendering: Tools for visualization.
 - edgevis_nodes_v1: Implementation of naive Edgevis
 - edgevis_nodes_v2: Implementation of edgevis with computed always visible vertices
 - edgevis_nodes_v3: Implementation of edgevis with online pruning
