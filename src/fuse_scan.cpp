#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
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
        FusedScan(ros::NodeHandle nh) : nh_(nh),
                                        sync_(MySyncPolicy(20), scan_front_, scan_back_)
        {   
            std::vector<double> polygon_x, polygon_y;
            if (!nh_.param<std::string>("scan_front_topic_name",
                                                scan_front_topic_name_,
                                                "/scan/scan_front")) {
                ROS_WARN_STREAM("Did not load scan_front_topic_name. Standard value is: " << scan_front_topic_name_);
            }
            if (!nh_.param<std::string>("scan_back_topic_name",
                                                scan_back_topic_name_,
                                                "/scan/scan_back")) {
                ROS_WARN_STREAM("Did not load scan_back_topic_name. Standard value is: " << scan_back_topic_name_);
            }
            if (!nh_.param<std::string>("fused_scan_topic_name",
                                                fused_scan_topic_name_,
                                                "/scan/fused_scan")) {
                ROS_WARN_STREAM("Did not load fused_scan_topic_name. Standard value is: " << fused_scan_topic_name_);
            }
            if (!nh_.param("robot_width", robot_width_, 0.6)) {
                ROS_WARN_STREAM("Did not load robot_width. Standard value is: " << robot_width_);
            }
            if (!nh_.param("robot_length", robot_length_, 0.65)) {
                ROS_WARN_STREAM("Did not load robot_length. Standard value is: " << robot_length_);
            }
            if (!nh_.param<std::string>("base_link",
                                                base_link_,
                                                "base_link")) {
                ROS_WARN_STREAM("Did not load base_link. Standard value is: " << base_link_);
            }
            nh_.getParam("polygon_x", polygon_x);
            nh_.getParam("polygon_y", polygon_y);

            for (size_t i = 0; i < std::min(polygon_x.size(), polygon_y.size()); i++) {
                Point p(polygon_x[i], polygon_y[i]);
                polygon_.push_back(p);
            }

            scan_front_.subscribe(nh_, scan_front_topic_name_, 20);
            scan_back_.subscribe(nh_, scan_back_topic_name_, 20);

            //coordinate callback for both laser scan message and a non_leg_clusters message
            sync_.registerCallback(boost::bind(&FusedScan::fused_scan_callback, this, _1, _2));

            //publish fused_scan to scan topic
            fused_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan> (fused_scan_topic_name_, 20);
            
            for (int i=0 ; i < 2160; i++){
                scan_fuse.ranges.push_back(40.0);
            }
            for (int i=0 ; i < 2160; i++){
                scan_fuse.intensities.push_back(100.0);
            }
        }

    private:

        ros::NodeHandle nh_;
        tf::TransformListener tflistener_;
        laser_geometry::LaserProjection projector_;

        message_filters::Subscriber <sensor_msgs::LaserScan> scan_front_;
        message_filters::Subscriber <sensor_msgs::LaserScan> scan_back_;

        typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync_;    
        ros::Publisher fused_scan_pub_;

        sensor_msgs::PointCloud cloud_fuse;
        sensor_msgs::LaserScan scan_fuse;

        std::string scan_front_topic_name_;
        std::string scan_back_topic_name_;
        std::string fused_scan_topic_name_;

        std::string base_link_;
        std::vector<Point> polygon_;

        double robot_length_;
        double robot_width_;

        //callback function
        void fused_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_front, 
                                 const sensor_msgs::LaserScan::ConstPtr& scan_back)
        {
            if (!tflistener_.waitForTransform(scan_front->header.frame_id, "base_link", 
                scan_front->header.stamp + ros::Duration().fromSec(scan_front->ranges.size()*scan_front->time_increment), 
                ros::Duration(3.0))) {
                    return;
            }
            
            if (!tflistener_.waitForTransform(scan_back->header.frame_id, "base_link", 
                scan_back->header.stamp + ros::Duration().fromSec(scan_back->ranges.size()*scan_back->time_increment), 
                ros::Duration(3.0))) {
                    return;
            }
            //convert scan_front to pointcloud
            sensor_msgs::PointCloud cloud_front;
            projector_.transformLaserScanToPointCloud("base_link", *scan_front, cloud_front, tflistener_);
            
            //convert scan_Back to pointcloud    
            sensor_msgs::PointCloud cloud_back;
            projector_.transformLaserScanToPointCloud("base_link", *scan_back, cloud_back, tflistener_);

            cloud_fuse.header.frame_id = cloud_front.header.frame_id;
            cloud_fuse.header.seq      = cloud_front.header.seq;
            cloud_fuse.header.stamp    = cloud_front.header.stamp;

            //merge the point clouds
            cloud_fuse.points = cloud_back.points;
            cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_front.points.begin(), cloud_front.points.end());
	
            for (int i=0 ; i < 2160; i++){
                scan_fuse.ranges[i]= 40.0;
            }
            for (long int i = 0; i < cloud_fuse.points.size(); i++){
                Point p(cloud_fuse.points[i].x, cloud_fuse.points[i].y);
                if (isInside(polygon_, p)) {
                    continue;
                }

                float angle = atan2f32(cloud_fuse.points[i].y, cloud_fuse.points[i].x);
                float range_val = hypotf32(cloud_fuse.points[i].x, cloud_fuse.points[i].y);
                int index = (int)((angle + M_PIf32)/scan_front->angle_increment);
                if (range_val < scan_fuse.ranges[index])
                    scan_fuse.ranges[index] = range_val;
                
            }
            scan_fuse.header.frame_id = cloud_fuse.header.frame_id;
            scan_fuse.header.stamp    = cloud_fuse.header.stamp;
            scan_fuse.header.seq      = cloud_fuse.header.seq;
            scan_fuse.angle_increment = scan_front->angle_increment;
            scan_fuse.angle_max       = M_PIf32;
            scan_fuse.angle_min       = -1* M_PIf32;
            scan_fuse.range_max       = scan_front->range_max;
            scan_fuse.range_min       = scan_front->range_min;
            scan_fuse.scan_time       = scan_front->scan_time;
            scan_fuse.time_increment  = scan_front->time_increment;
            fused_scan_pub_.publish(scan_fuse);
        }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_fusion_node");
    ros::NodeHandle nh;
    FusedScan fs(nh);
    ros::spin();
}
