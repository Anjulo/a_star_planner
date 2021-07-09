# a_star_planner

This package implements the A* star path-planning algorithm in c++ and integrate with the ROS environment.

More information on A* algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm

To reproduce:
1. Play rosbag file named as "Gridmap_and_points.bag" and use occupancy grid map topic.
    - Gridmap_and_points.bag (Credits: KAIST Unmanned System Research Group)
         - /semantics/costmap_generator/occupancy_grid : nav_msgs/OccupancyGrid.msg
2. Set goal pose using RVIZ 2D Nav Goal tool. 