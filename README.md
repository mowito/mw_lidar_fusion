# mw_lidar_fusion

mw_lidar_fusion is a tool for fusing LiDAR and PointCloud2 data from upto 4 sources each.

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

A static transform between the `lidar` frame of reference and the `base_link` frame of reference is to be published for all lidars.

```
 <!-- static transform between laser and the robot -->
    <node pkg="tf" type="static_transform_publisher" name="base_2_lidar" args="0 0 0 0 0 0 base_link laser 670"/>
```    
Edit the parameters in the line above and add it to your launch file if a static tf is not being published through other means.

## Parameters in config.yaml

- scan_topics: list of all LaserScan topics, max 4

- pointcloud2_topics: list of PointCloud2 topics, max 4

- fused_scan_topic_name: topic name at which fused LaserScan will be published

- fused_pointcloud_topic_name: topic name at which fused PointCloud2 will be published

- base_link: Name of base_link of the robot

- polygon: coordinates of edges of the polygon to be cropped from the Laserscan/Pointcloud data

- angle_min: minimum angle for which data from all input lidar's is included in the fused scan

- angle_max: maximum angle for which data from all input lidar's is included in the fused scan
- polygon_topic_name: topic name at which coordinates of edges of the polygon will be published

## Example

The points in white represent the filtered and fused data whereas the points in red represent the unfiltered raw laserscan

Fusion with single LiDAR and angle_min=-pi/2 angle_max=pi/2

![](/examples/1.png)

Fusion with single LiDAR and angle_min= pi/2 angle_max=pi

![](/examples/4.png)