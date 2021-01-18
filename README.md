# mw_lidar_fusion

mw_lidar_fusion is a tool for fusing LiDAR data from two sources.

## Build instructions
```
mkdir -p mowito_ws/src && cd mowito_ws/src
git clone git@github.com:mowito/mw_lidar_fusion.git
cd ..
source /opt/ros/noetic/setup.bash
catkin build
source ./devel/setup.bash
```

You can then launch different configurations by:
`roslaunch mowito_difacto <file_name>.launch`
