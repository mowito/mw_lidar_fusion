#include <fuse_scan.hpp>

FusedScan::FusedScan(ros::NodeHandle nh) :
nh_(nh), private_nh_("~"),scan_pointcloud_sync_(ScanPointCloudPolicy(20), scan_front_, scan_back_, cloud_front_, cloud_back_),
scan_sync_(ScanPolicy(20), scan_front_, scan_back_), pointcloud_sync_(PointCloudPolicy(20), cloud_front_, cloud_back_)
{
    XmlRpc::XmlRpcValue polygon;
    if (!private_nh_.param<int>("num_lidars",
                                num_lidars_,
                                0)) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load num_lidars. Setting default to: "<< num_lidars_);
    }
    if (!private_nh_.param<int>("num_depth_sensors",
                                num_depth_sensors_,
                                0)) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load num_depth_sensors. Setting default to: "<< num_depth_sensors_);
    }
    if (!private_nh_.param<std::string>("scan_front_topic_name",
                                scan_front_topic_name_,
                                "/scan/scan_front") && num_lidars_) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load scan_front_topic_name. Standard value is: " << scan_front_topic_name_);
    }
    if (!private_nh_.param<std::string>("scan_back_topic_name",
                                scan_back_topic_name_,
                                "/scan/scan_back") && num_lidars_) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load scan_back_topic_name. Standard value is: " << scan_back_topic_name_);
        scan_back_topic_name_ = scan_front_topic_name_;
    }
    if (!private_nh_.param<std::string>("cloud_back_topic_name",
                                cloud_back_topic_name_,
                                "/cloud/pointcloud_back") && num_depth_sensors_) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load cloud_back_topic_name. Standard value is: " << cloud_back_topic_name_);
    }
    if (!private_nh_.param<std::string>("cloud_front_topic_name",
                                cloud_front_topic_name_,
                                "/cloud/pointcloud_front") && num_depth_sensors_) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load cloud_front_topic_name. Standard value is: " << cloud_front_topic_name_);
        cloud_back_topic_name_ = cloud_front_topic_name_;
    }
    if (!private_nh_.param<std::string>("fused_scan_topic_name",
                                fused_scan_topic_name_,
                                "/scan/fused_scan")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load fused_scan_topic_name. Standard value is: " << fused_scan_topic_name_);
    }
    if (!private_nh_.param<std::string>("fused_pointcloud_topic_name",
                                fused_pointcloud_topic_name_,
                                "/cloud/fused_pointcloud")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load fused_pointcloud_topic_name. Standard value is: " << fused_pointcloud_topic_name_);
    }
    if (!private_nh_.param<std::string>("base_link",
                                base_link_,
                                "base_link")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load base_link. Standard value is: " << base_link_);
    }
    if (!private_nh_.getParam("polygon", polygon)) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load polygon");
    }
    if (!private_nh_.param<std::string>("polygon_topic_name",
                                polygon_topic_name_,
                                "/lidar_crop_polygon")) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load polygon_topic_name. Standard value is: " << polygon_topic_name_);
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
    cloud_front_.subscribe(nh_, cloud_front_topic_name_, 20);
    cloud_back_.subscribe(nh_, cloud_back_topic_name_, 20);

    if(num_lidars_ > 0 && num_depth_sensors_ > 0)
      scan_pointcloud_sync_.registerCallback(boost::bind(&FusedScan::fusedScanCloudCallback, this, _1, _2, _3, _4));
    else if(num_depth_sensors_>0)
      pointcloud_sync_.registerCallback(boost::bind(&FusedScan::fusedCloudCallback, this, _1, _2));
    else
      scan_sync_.registerCallback(boost::bind(&FusedScan::fusedScanCallback, this, _1, _2));

    //coordinate callback for both laser scan message and a non_leg_clusters message
    // sync_.registerCallback(boost::bind(&FusedScan::fusedScanCallback, this, _1, _2));

    //publish fused_scan to scan topic
    fused_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan> (fused_scan_topic_name_, 20);
    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped> (polygon_topic_name_, 5);
    fused_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (fused_pointcloud_topic_name_, 20);

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
      if(num_lidars_ > 1)
          projector_.transformLaserScanToPointCloud(base_link_, *scan_back, cloud_back, tflistener_);

      cloud_fuse.header.frame_id = cloud_front.header.frame_id;
      cloud_fuse.header.seq      = cloud_front.header.seq;
      cloud_fuse.header.stamp    = cloud_front.header.stamp;
      //merge the point clouds
      cloud_fuse.points = cloud_front.points;

      if(num_lidars_ > 1)
          cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_back.points.begin(), cloud_back.points.end());

      mergePointClouds(cloud_fuse, scan_front->angle_increment);
      sendVisualization(scan_front);
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<< e.what());
    }
}

void FusedScan::fusedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_front,
                                    const sensor_msgs::PointCloud2::ConstPtr& cloud_back)
{

    if (!tflistener_.waitForTransform(cloud_front->header.frame_id, base_link_,
        cloud_front->header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }

    if (!tflistener_.waitForTransform(cloud_back->header.frame_id, base_link_,
        cloud_back->header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }
    //tranform to base_link frame
    sensor_msgs::PointCloud2 tf_cloud_front, tf_cloud_back;
    pcl_ros::transformPointCloud (base_link_, *cloud_front, tf_cloud_front, tflistener_);
    pcl_ros::transformPointCloud (base_link_, *cloud_back, tf_cloud_back, tflistener_);
    sensor_msgs::PointCloud cloud_temp;

    cloud_fuse.header.frame_id = tf_cloud_front.header.frame_id;
    cloud_fuse.header.seq      = cloud_front->header.seq;
    cloud_fuse.header.stamp    = cloud_front->header.stamp;
    //merge the point clouds
    convertPointCloud2ToPointCloud(tf_cloud_front, cloud_temp);
    cloud_fuse.points = cloud_temp.points;
    if(num_depth_sensors_ > 1){
        convertPointCloud2ToPointCloud(tf_cloud_back, cloud_temp);
        cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_temp.points.begin(), cloud_temp.points.end());
}
    mergePointClouds(cloud_fuse, 0.0);
    sendCloudVisualization();
    sendPolygonVisualization();

}

void FusedScan::fusedScanCloudCallback(const sensor_msgs::LaserScan::ConstPtr& scan_front, const sensor_msgs::LaserScan::ConstPtr& scan_back,
                                    const sensor_msgs::PointCloud2::ConstPtr& cloud_front, const sensor_msgs::PointCloud2::ConstPtr& cloud_back)
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

    if (!tflistener_.waitForTransform(cloud_front->header.frame_id, base_link_,
        cloud_front->header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }

    if (!tflistener_.waitForTransform(cloud_back->header.frame_id, base_link_,
        cloud_back->header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }
    //tranform to base_link frame
    sensor_msgs::PointCloud2 tf_cloud_front, tf_cloud_back;

    //convert scan_front to pointcloud
    sensor_msgs::PointCloud scan_cloud_front;
    try
    {
      projector_.transformLaserScanToPointCloud(base_link_, *scan_front, scan_cloud_front, tflistener_);
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
    }

    //convert scan_Back to pointcloud
    sensor_msgs::PointCloud scan_cloud_back, cloud_temp;
    try
    {
      if(num_lidars_ > 1)
          projector_.transformLaserScanToPointCloud(base_link_, *scan_back, scan_cloud_back, tflistener_);

      pcl_ros::transformPointCloud (base_link_, *cloud_front, tf_cloud_front, tflistener_);

      if(num_depth_sensors_ > 1)
        pcl_ros::transformPointCloud (base_link_, *cloud_back, tf_cloud_back, tflistener_);

      cloud_fuse.header.frame_id = scan_cloud_front.header.frame_id;
      cloud_fuse.header.seq      = scan_cloud_front.header.seq;
      cloud_fuse.header.stamp    = scan_cloud_front.header.stamp;
      //merge the point clouds
      cloud_fuse.points = scan_cloud_front.points;
      if(num_lidars_ > 1)
          cloud_fuse.points.insert(cloud_fuse.points.end(), scan_cloud_back.points.begin(), scan_cloud_back.points.end());

      convertPointCloud2ToPointCloud(tf_cloud_front, cloud_temp);
      cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_temp.points.begin(), cloud_temp.points.end());

      if(num_depth_sensors_ > 1 ){
        convertPointCloud2ToPointCloud(tf_cloud_front, cloud_temp);
        cloud_fuse.points.insert(cloud_fuse.points.end(), cloud_temp.points.begin(), cloud_temp.points.end());
      }
      mergePointClouds(cloud_fuse, scan_front->angle_increment);
      sendVisualization(scan_front);
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<< e.what());
    }
}

void FusedScan::mergePointClouds(sensor_msgs::PointCloud& cloud_fuse, double angle_increment) {

    cloud_crop_.header.frame_id = cloud_fuse.header.frame_id;
    cloud_crop_.header.seq      = cloud_fuse.header.seq;
    cloud_crop_.header.stamp    = cloud_fuse.header.stamp;
    cloud_crop_.points.clear();

    for (int i = 0 ; i < 2160; i++){
        scan_fuse.ranges[i]= 40.0;
    }
    for (long int i = 0; i < cloud_fuse.points.size(); i++){

        // check if point inside polygon
        Point cloud_point = Point(cloud_fuse.points[i].x,cloud_fuse.points[i].y);
        if(CGAL::bounded_side_2(polygon_.begin(), polygon_.end(), cloud_point, K()) == CGAL::ON_BOUNDED_SIDE){
          continue;
        }
        cloud_crop_.points.push_back(cloud_fuse.points[i]);

        if(angle_increment > 0){
          float angle = atan2f32(cloud_fuse.points[i].y, cloud_fuse.points[i].x);
          float range_val = hypotf32(cloud_fuse.points[i].x, cloud_fuse.points[i].y);
          int index = (int)((angle + M_PIf32)/angle_increment);
          if (range_val < scan_fuse.ranges[index])
              scan_fuse.ranges[index] = range_val;
          }

    }

}

void FusedScan::sendLaserVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front) {
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

    fused_scan_pub_.publish(scan_fuse); //publish fused scan

}

void FusedScan::sendCloudVisualization() {

  //publish PointCloud2
  sensor_msgs::PointCloud2 fused_cloud;
  convertPointCloudToPointCloud2(cloud_crop_, fused_cloud);
  fused_pointcloud_pub_.publish(fused_cloud);

}

void FusedScan::sendPolygonVisualization(){  // update polygon timestamp
  polygon_viz_.header.frame_id = base_link_;
  polygon_viz_.header.stamp = cloud_fuse.header.stamp;

  polygon_pub_.publish(polygon_viz_); //publish polygon

}

void FusedScan::sendVisualization(const sensor_msgs::LaserScan::ConstPtr& scan_front){

  sendLaserVisualization(scan_front);
  sendCloudVisualization();
  sendPolygonVisualization();

}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_fusion_node");
    ros::NodeHandle nh;
    FusedScan fs(nh);
    ros::spin();
}
