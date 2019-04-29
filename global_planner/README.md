This is a global_planner using RRT* algorithm.

We offer 4 kinds of algorithms in 
* global_planner/src/planner_core.cpp

You can choose 1 particular algorithm by annotating other 3 algorithms.
========================================================================
1. A*:
also need ~/catkin_ws/src/nav_sim/cfg/Astar.yaml
<-->
2. RRT:
the simple RRT algorithm
<-->
3. RRT*:
set the R=7, if R is too big, the path will be smoothier but may cross the obstacles
if R is 0, is the simple RRT
<-->
4. better RRT*:
10000 iteration after running RRT* to find a path, in order to get a much better path, but will cost a long time
