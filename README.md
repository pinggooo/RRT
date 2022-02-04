# RRT Algorithm for Robot path planning
```
This is an Intern program Project.

- ROS2 foxy
- C++ 11
- Turtlebot3 packages
- Eigen3
- Catch2
- Rviz
- Gazebo
- Navigation2
```

## Usage
#### Compile
```
$ cd ~/xxx_ws/src
$ git clone https://github.com/pinggooo/RRT
$ cd ~/xxx_ws
$ colcon build --symlink-install
```

#### Execution
```
$ ros2 run rrt rrt_test
```
```
$ cd ~/xxx_ws/src/turtlebot3/turtlebot3_navigation2/map
$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="turtlebot3_world.yaml"
```
```
$ cd ~/xxx_ws/src/turtlebot3/turtlebot3_navigation2/map
$ ros2 run nav2_util lifecycle_bringup map_server
```


<p align="center"><img src="https://user-images.githubusercontent.com/69897315/152476136-9965fead-aa63-4eb9-8fe7-acd3ec788dd7.png" width="400" height="300"></p>


###### <div align="right">Intern Associate, Robot Intelligence Team, Samsung Research</div>
