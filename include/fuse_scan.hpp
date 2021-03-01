#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <xmlrpcpp/XmlRpcValue.h> 

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#define EPSI 10e-6

class Point {
  public:
  Point(double _x, double _y) {
      x = _x;
      y = _y;
  }

  double x;
  double y;
};

bool onSegment(Point p, Point q, Point r) { 
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && 
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) 
        return true; 
    return false; 
}

int orientation(Point p, Point q, Point r) { 
    int val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 
 
    if (val <= EPSI)
      return 0;
    return (val > 0)? 1: 2;
}

bool doIntersect(Point p1, Point q1, Point p2, Point q2) { 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
 
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
 
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1))
      return true; 
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1))
      return true; 
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2))
      return true; 
 
    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2))
      return true; 
 
    return false;
} 

bool isInside(std::vector<Point> &polygon, Point p) {
    if (polygon.size() < 3) return false; 
 
    Point extreme = {std::numeric_limits<double>::max(), p.y}; 

    int count = 0, i = 0; 
    do { 
        int next = (i+1)%polygon.size(); 

        if (doIntersect(polygon[i], polygon[next], p, extreme)) {
            if (orientation(polygon[i], p, polygon[next]) == 0) 
            return onSegment(polygon[i], p, polygon[next]); 
 
            count++; 
        } 
        i = next; 
    } while (i != 0); 
 
    return count%2;
}

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
    std::vector<Point> polygon_;

    double robot_length_;
    double robot_width_;
};
