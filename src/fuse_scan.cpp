#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <algorithm>
#include <math.h>


class FusedScan {

  public:
        FusedScan(ros::NodeHandle nh) : nh_(nh),
                                        sync_(MySyncPolicy(20), scan_front_, scan_back_)
        {   
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

                if ((cloud_fuse.points[i].x > -robot_length_/2 && cloud_fuse.points[i].x < robot_length_/2) &&
                    (cloud_fuse.points[i].y > -robot_width_/2 && cloud_fuse.points[i].y < robot_width_/2)){
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
