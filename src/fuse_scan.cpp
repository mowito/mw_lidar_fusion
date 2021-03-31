#include <fuse_scan.hpp>

FusedScan::FusedScan(ros::NodeHandle nh) :
nh_(nh), sync_(MySyncPolicy(20), scan_front_, scan_back_)
{
    XmlRpc::XmlRpcValue polygon;
    if (!nh_.param<std::string>("scan_front_topic_name",
                                scan_front_topic_name_,
                                "/scan/scan_front")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load scan_front_topic_name. Standard value is: " << scan_front_topic_name_);
    }
    if (!nh_.param<std::string>("scan_back_topic_name",
                                scan_back_topic_name_,
                                "/scan/scan_back")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load scan_back_topic_name. Standard value is: " << scan_back_topic_name_);
    }
    if (!nh_.param<std::string>("fused_scan_topic_name",
                                fused_scan_topic_name_,
                                "/scan/fused_scan")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load fused_scan_topic_name. Standard value is: " << fused_scan_topic_name_);
    }

    if (!nh_.param<std::string>("base_link",
                                base_link_,
                                "base_link")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load base_link. Standard value is: " << base_link_);
    }
    if (!nh_.getParam("polygon", polygon)) {
        ROS_ERROR_STREAM("[LIDAR FUSION] Did not load polygon");
    }
    if (!nh_.param<std::string>("polygon_topic_name",
                                polygon_topic_name_,
                                "/lidar_crop_polygon")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load polygon_topic_name. Standard value is: " << polygon_topic_name_);
    }

    if (scan_front_topic_name_ == scan_back_topic_name_) {

        single_lidar_ = true;
    } else {

        single_lidar_ = false;
    }

    if (polygon.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < polygon.size(); i++) {
                polygon_.push_back(Point(polygon[i][0], polygon[i][1]));
                geometry_msgs::Point32 pt;
                pt.x = double(polygon[i][0]);
                pt.y = double(polygon[i][1]);
                polygon_viz_.polygon.points.push_back(pt);
                // polygon_viz_.header.frame_id = base_link_;
            }
        }

    // check if convex polygon
    typedef CGAL::Polygon_2<K> Polygon_2;
    if(!Polygon_2(polygon_.begin(),polygon_.end()).is_convex()){
      ROS_ERROR_STREAM("[LIDAR FUSION] Polygon is invalid");
      polygon_.clear();
    }


    scan_front_.subscribe(nh_, scan_front_topic_name_, 20);
    scan_back_.subscribe(nh_, scan_back_topic_name_, 20);
    //coordinate callback for both laser scan message and a non_leg_clusters message
    sync_.registerCallback(boost::bind(&FusedScan::fusedScanCallback, this, _1, _2));

    //publish fused_scan to scan topic
    fused_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan> (fused_scan_topic_name_, 20);
    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped> (polygon_topic_name_, 5);

    for (int i = 0 ; i < 2160; i++){
        scan_fuse.ranges.push_back(40.0);
    }
    for (int i = 0 ; i < 2160; i++){
        scan_fuse.intensities.push_back(100.0);
    }
}

//callback function
void FusedScan::fusedScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_front,
                                    const sensor_msgs::LaserScan::ConstPtr& scan_back)
{

    if (!tflistener_.waitForTransform(scan_front->header.frame_id, base_link_,
        scan_front->header.stamp + ros::Duration().fromSec(scan_front->ranges.size()*scan_front->time_increment),
        ros::Duration(3.0))) {
        return;
    }

    if (!tflistener_.waitForTransform(scan_back->header.frame_id, base_link_,
        scan_back->header.stamp + ros::Duration().fromSec(scan_back->ranges.size()*scan_back->time_increment),
        ros::Duration(3.0))) {
        return;
    }
    //convert scan_front to pointcloud
    sensor_msgs::PointCloud cloud_front;
    try
    {
      projector_.transformLaserScanToPointCloud(base_link_, *scan_front, cloud_front, tflistener_);
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
    }

    //convert scan_Back to pointcloud
    sensor_msgs::PointCloud cloud_back;

    try
    {
      if(!single_lidar_)
          projector_.transformLaserScanToPointCloud(base_link_, *scan_back, cloud_back, tflistener_);

      mergePointClouds(cloud_front, cloud_back, scan_front);
      sendVisualization(scan_front);
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<< e.what());
    }

}


void FusedScan::mergePointClouds(sensor_msgs::PointCloud& cloud_front,
                                 sensor_msgs::PointCloud& cloud_back,
                                 const sensor_msgs::LaserScan::ConstPtr& scan_front) {
    cloud_fuse.header.frame_id = cloud_front.header.frame_id;
    cloud_fuse.header.seq      = cloud_front.header.seq;
    cloud_fuse.header.stamp    = cloud_front.header.stamp;

    //merge the point clouds
    cloud_fuse.points = cloud_front.points;

    if(!single_lidar_)
        cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_back.points.begin(), cloud_back.points.end());

    for (int i = 0 ; i < 2160; i++){
        scan_fuse.ranges[i]= 40.0;
    }
    for (long int i = 0; i < cloud_fuse.points.size(); i++){

        // if ((cloud_fuse.points[i].x < (robot_length_/2 + x_offset_)) && (cloud_fuse.points[i].x > (-robot_length_/2 + x_offset_)) &&
        //     (cloud_fuse.points[i].y < (robot_width_/2 + y_offset_)) && (cloud_fuse.points[i].y > (-robot_width_/2 + y_offset_))) {
        //         continue;
        // }

        // check if point inside polygon
        Point cloud_point = Point(cloud_fuse.points[i].x,cloud_fuse.points[i].y);
        if(CGAL::bounded_side_2(polygon_.begin(), polygon_.end(), cloud_point, K()) == CGAL::ON_BOUNDED_SIDE){
          continue;
        }

        float angle = atan2f32(cloud_fuse.points[i].y, cloud_fuse.points[i].x);
        float range_val = hypotf32(cloud_fuse.points[i].x, cloud_fuse.points[i].y);
        int index = (int)((angle + M_PIf32)/scan_front->angle_increment);
        if (range_val < scan_fuse.ranges[index])
            scan_fuse.ranges[index] = range_val;

    }

}

void FusedScan::sendVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front) {
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

    // update polygon timestamp
    polygon_viz_.header.frame_id = base_link_;
    polygon_viz_.header.stamp = cloud_fuse.header.stamp;

    fused_scan_pub_.publish(scan_fuse); //publish fused scan
    polygon_pub_.publish(polygon_viz_); //publish polygon
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_fusion_node");
    ros::NodeHandle nh;
    FusedScan fs(nh);
    ros::spin();
}
