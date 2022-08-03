# Sprint august 2022
Logger for daily progress on sprint during august 2022.

### 3.8.2022
- Compiled succesfully robust geometry from trivis. Hopefully with all parts needed. 
Problem seems to be in line parallelism. Have to find why that occurs, and find general solution.

### 2.8.2022
- Copied robust geometry from trivis to edgevis. Now making it work.


### 1.8.2022
 - Found most likely cause of nan bug. Line intersection divides by 0 -> inf / nan. Robust math should fix this,
so first step is taking that from TriVis. Hopefully it will solve this issue.