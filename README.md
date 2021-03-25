# mw_lidar_fusion

mw_lidar_fusion is a tool for fusing LiDAR data from two sources.

## Build instructions
```
sudo apt-get install libcgal-dev
mkdir -p mowito_ws/src && cd mowito_ws/src
git clone git@github.com:mowito/mw_lidar_fusion.git
cd ..
source /opt/ros/noetic/setup.bash
catkin build
source ./devel/setup.bash
```

You can then launch different configurations by:
`roslaunch lidar_fusion lidar_fusion.launch`
