#ifndef FUSE_SCAN_H
#define FUSE_SCAN_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2/utils.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#define EPSI 10e-6

class FusedScan {
  public:
  FusedScan(ros::NodeHandle nh);

  private:
  /**
   * @brief callback function for fusing upto 4 laserscan topics
   * @param The LaserScan messages from the topics
   */
  void fusedScanCallback(const sensor_msgs::LaserScan::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&,
                          const sensor_msgs::LaserScan::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&);


  /**
   * @brief callback function for fusing upto 4 pointcloud topics
   * @param The pointcloud2 messages from the topics
   */
  void fusedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&,
                            const sensor_msgs::PointCloud2::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&);


  /**
   * @brief callback function for fusing upto 4 laserscan and 4 pointcloud topics
   * @param The LaserScan and PointCloud2 messages from the topics
   */
  void fusedScanCloudCallback(const sensor_msgs::LaserScan::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&,
                                const sensor_msgs::LaserScan::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&,
                                  const sensor_msgs::PointCloud2::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&,
                                    const sensor_msgs::PointCloud2::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&);

  /**
   * @brief crops fused pointcloud and converts to laserscan
   * @param angle increment parameter of lidar
   */
  void mergePointClouds(double angle_increment);

  /**
   * @brief publishes LaserScan, PointCloud2 and polygon
   * @param laserscan to publish
   */
  void sendVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front);

  /**
   * @brief publishes LaserScan
   * @param laserscan to publish
   */
  void sendLaserVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front);

  void sendCloudVisualization(); //publishes fused pointcloud
  void sendPolygonVisualization(); //publishes polygon

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tflistener_;
    laser_geometry::LaserProjection projector_;
    int max_topic_num_ = 4; //max num of topics that can be fused for each type

    //vector of message filter subscribers for LaserScan and PointCloud2
    std::vector<message_filters::Subscriber <sensor_msgs::LaserScan>> scan_sub_ = std::vector<message_filters::Subscriber <sensor_msgs::LaserScan>>(max_topic_num_);
    std::vector<message_filters::Subscriber <sensor_msgs::PointCloud2>> cloud_sub_ =  std::vector<message_filters::Subscriber <sensor_msgs::PointCloud2>>(max_topic_num_);

    std::vector<sensor_msgs::LaserScan> scan_msg_;
    std::vector<sensor_msgs::PointCloud2> cloud2_msg_;

    //ApproximateTime policy for syncing upto 4 LaserScan and 4 PointCloud2 topics
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::LaserScan, sensor_msgs::LaserScan,
                                                                sensor_msgs::LaserScan, sensor_msgs::LaserScan,
                                                                  sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                                    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ScanPointCloudPolicy;

    //ApproximateTime policy for syncing upto 4 laserscan topics
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::LaserScan, sensor_msgs::LaserScan,
                                                                sensor_msgs::LaserScan, sensor_msgs::LaserScan> ScanPolicy;

    //ApproximateTime policy for syncing upto 4 pointcloud2 topics
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                                sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PointCloudPolicy;

    message_filters::Synchronizer<ScanPointCloudPolicy> scan_pointcloud_sync_;
    message_filters::Synchronizer<ScanPolicy> scan_sync_;
    message_filters::Synchronizer<PointCloudPolicy> pointcloud_sync_;

    ros::Publisher fused_scan_pub_; //publisher for fused scan
    ros::Publisher fused_pointcloud_pub_; //publisher for fused PointCloud2
    ros::Publisher polygon_pub_; //publisher for polygon

    sensor_msgs::PointCloud cloud_fuse_, cloud_crop_;
    sensor_msgs::LaserScan scan_fuse_;

    std::vector<std::string> scan_topics_, cloud_topics_; //vector of scan and pointcloud2 topic names to subscribe to
    std::string fused_scan_topic_name_;
    std::string fused_pointcloud_topic_name_;
    std::string polygon_topic_name_;
    std::string base_link_;

    //CGAL polygon and point types to check if point lies within polygon
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point;
    std::vector<Point> polygon_;
    geometry_msgs::PolygonStamped polygon_viz_;

    int num_lidars_; //number of scan topics
    int num_depth_sensors_;//number of PointCloud2 topics

};

#endif
