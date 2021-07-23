# mw_lidar_fusion

mw_lidar_fusion is a tool for fusing and filtering LiDAR and PointCloud2 data from upto 4 sources each.

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

## Testing On Simulation

If you are testing the stack in simulation using rosbags you will need to sync the clocks for the bag and the sim.

Add this line to the roslaunch file
 ```
 <param name="use_sim_time" value="true"/>
```

And play your rosbag's using the `--clock` parameter.
```
rosbag play --clock name_of_bag.bag 
```

## Example

### One LiDAR

The points in white represent the filtered and fused data whereas the points in red represent the unfiltered raw laserscan

- Filtering with angle_min=-pi/2 angle_max=pi/2

![](/examples/1.png)

- Filtering with angle_min= pi/2 angle_max=pi

![](/examples/5.png)

### Two LiDAR's

The points in white represent the filtered and fused data whereas the points in red and purple represent the unfiltered raw laserscan from two seperate rostopics

- Raw data

![](/examples/8.png)

- Fusion of LiDAR Data

![](/examples/6.png)

- Fusion and filtering with angle_min= 0 angle_max=pi

![](/examples/7.png)
