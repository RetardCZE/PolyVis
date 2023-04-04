# Implementations

We are separating our library into 3 parts. geomMesh folder contains implementation of parsers from Lukas Fanta.
These parsers are interface for Fade2D library and we used them during our research.
All meshes have to be converted to GeomMesh structure and then passed either to PolyVis object or to edgemesh which 
does some additional computation and passes it to EdgeVis object.

In folder polyanya is the reduced implementation of polyanya project. The implementation provides object PolyVis 
with method get_visibility_polygon() for computation of visibility region for point. 
If a triangular mesh is passed, the library works as TEA if polygonal mesh is passed it works as PEA.

Folder edgevis contains our implementation of new algorithm. We started by copying the polyanya project, but when 
we finished, only few basic structures remained the same. 

Both polyanya and edgevis use 3rd party libraries for robust orientation test. (Clipper, Predicates, Triangle)

edgevis [README.md](edgevis/src/edgevis/README.md)

polyanya [README.md](polyanya/src/polyanya/README.md)