# Polyanya

This library took polyanya project, removed unnecessary code and added robust orientation tests.
It is not copy of the polyanya project, so if you want to use their project, download a new copy!

Our implementation is mainly in folder search. Interface for robust test was taken from TriVis project of Jan Mikula (intersections.cc and robust_geometry.cc)
and all was put together in polyvis.cpp (Thats the only file here touched only be me - Jakub Rosol)

The main part is in the expansion.cpp. It is the implementation for single expansion step of TEA and PEA. I've removed
unnecessary parts and added possibility to use orientation test from R. J. Shewchuk.

## Citation and License from polyanya project

```
@inproceedings{DBLP:conf/ijcai/CuiHG17,
  author    = {Michael Cui and
               Daniel Damir Harabor and
               Alban Grastien},
  title     = {Compromise-free Pathfinding on a Navigation Mesh},
  booktitle = {Proceedings of the Twenty-Sixth International Joint Conference on
               Artificial Intelligence, {IJCAI} 2017, Melbourne, Australia, August
               19-25, 2017},
  pages     = {496--502},
  year      = {2017},
  crossref  = {DBLP:conf/ijcai/2017},
  url       = {https://doi.org/10.24963/ijcai.2017/70},
  doi       = {10.24963/ijcai.2017/70},
  timestamp = {Wed, 27 Jun 2018 12:24:11 +0200},
  biburl    = {https://dblp.org/rec/bib/conf/ijcai/CuiHG17},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

This implementation of Polyanya is licensed under MIT. Several source files from
Daniel Harabor's [Warthog project] were used this project - these files are also
licensed under MIT.
These files are:
`helpers/cfg.cpp`, `helpers/cfg.h`, `helpers/cpool.h`, `helpers/timer.cpp` and
`helpers/timer.h`.

Fade2D is used to generate triangulations for use with this
with this implementation. Please note that commercial use of Fade2D requires
a valid commercial license.

[paper]: http://www.ijcai.org/proceedings/2017/0070.pdf
[Warthog project]: https://bitbucket.org/dharabor/pathfinding
