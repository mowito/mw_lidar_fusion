#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>


#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#define EPSI 10e-6

class FusedScan {
  public:
  FusedScan(ros::NodeHandle nh);

  private:
  void fusedScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_front,
                                 const sensor_msgs::LaserScan::ConstPtr& scan_back);

  void mergePointClouds(sensor_msgs::PointCloud& cloud_front,
                        sensor_msgs::PointCloud& cloud_back,
                        const sensor_msgs::LaserScan::ConstPtr& scan_front);
  void sendVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front);

  private:
  ros::NodeHandle nh_;
    tf::TransformListener tflistener_;
    laser_geometry::LaserProjection projector_;

    message_filters::Subscriber <sensor_msgs::LaserScan> scan_front_;
    message_filters::Subscriber <sensor_msgs::LaserScan> scan_back_;


    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    ros::Publisher fused_scan_pub_;

    sensor_msgs::PointCloud cloud_back;
    sensor_msgs::PointCloud cloud_fuse;
    sensor_msgs::LaserScan scan_fuse;

    std::string scan_front_topic_name_;
    std::string scan_back_topic_name_;
    std::string fused_scan_topic_name_;
    bool single_lidar_;

    std::string base_link_;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point;
    std::vector<Point> polygon_;

    // double x_offset_;
    // double y_offset_;
};
