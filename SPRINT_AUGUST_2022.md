# Sprint august 2022
Logger for daily progress on sprint during august 2022.

### 6.8.2022 + 7.8.2022
- Implemented robust geometry. And started debugging edgevis.
- Got lost in debugging and best solution will probably be fast reimplementation with
better modularity. Need possibility of easy unit testing.

### 5.8.2022
- Free day

### 4.8.2022
- Implementing robust Line-Segment intersection. Seems that previous approach might have
 been logicaly correct, but all cases of intersecting lines/segments werent taken into
account. Once more robust handling of these intersections is done, results should stabilise.

### 3.8.2022
- Compiled succesfully robust geometry from trivis. Hopefully with all parts needed. 
Problem seems to be in line parallelism. Have to find why that occurs, and find general solution.

### 2.8.2022
- Copied robust geometry from trivis to edgevis. Now making it work.


### 1.8.2022
 - Found most likely cause of nan bug. Line intersection divides by 0 -> inf / nan. Robust math should fix this,
so first step is taking that from TriVis. Hopefully it will solve this issue.