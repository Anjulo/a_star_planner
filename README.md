## a_star_planner

This package implements the A* star path-planning algorithm in c++ and integrate with the ROS environment.

![](https://media.giphy.com/media/GZfVydwCMw4hjeprNM/giphy.gif)

More information on A* algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm

## To reproduce:
- Run roscore in one window of the terminal
``` 
roscore
```
- Run map server
```
cd map
rosrun map_server map_server map.yaml
```
- Run the rosbag (Credits: [KAIST Unmanned System Research Group](https://www.unmanned.kaist.ac.kr))
```
rosbag play --pause Gridmap_and_points.bag
# you can start and pause as needed then
```
- Run the a* path planner node
```
rosrun a_star_planner a_star_planner_node
```
- Set goal pose using RVIZ 2D Nav Goal tool. 
