# NDT C++

Minimum NDT

https://github.com/TakanoTaiga/ndt_cpp/assets/53041471/510656ef-73a8-49dd-b51f-698165e1922a



## RUN

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kazu-321/ndt_cpp_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 run ndt_cpp_ros2 ndt_cpp_ros2
```

## IO
### Input
- `/input/scan`:  sensor_msgs::msg::LaserScan
- `/input/map` :  nav_msgs::msg::OccupancyGrid

### Output
- `/output/pose`: geometry_msgs::msg::PoseStamped
- `/tf`: map -> odom

## Supposed TF
odom -> base_link -> laser_link

## Parameters
- `map_frame_id` : map frame id in tf. default: map
- `odom_frame_id` : odom frame id in tf. default: odom
- `base_frame_id` : base frame id in tf. default: base_link
- `laser_frame_id` : laser frame id in tf. default: laser_link

- `initial_pose_x` : initial pose x. default: 0.0
- `initial_pose_y` : initial pose y. default: 0.0
- `initial_pose_yaw` : initial pose yaw. default: 0.0

- `frequency_millisec` : frequency. default: 100

## LICENSE

NDT C++ specific code is distributed under Apache2.0 License.
The following extensions are included (*.cpp *.md)


NDT C++ specific data are distributed under MIT License.
The following extensions are included (*.txt)
