#include <fuse_scan.hpp>

FusedScan::FusedScan(ros::NodeHandle nh) :
nh_(nh), private_nh_("~"),
scan_pointcloud_sync_(ScanPointCloudPolicy(20), scan_sub_[0], scan_sub_[1], scan_sub_[2], scan_sub_[3],
                        cloud_sub_[0], cloud_sub_[1], cloud_sub_[2], cloud_sub_[3]),
scan_sync_(ScanPolicy(20), scan_sub_[0], scan_sub_[1], scan_sub_[2], scan_sub_[3]),
pointcloud_sync_(PointCloudPolicy(20), cloud_sub_[0], cloud_sub_[1], cloud_sub_[2], cloud_sub_[3]),
init_params_(false)
{
    XmlRpc::XmlRpcValue polygon;
    scan_topics_ = {};
    cloud_topics_ = {};
    if (!private_nh_.getParam("scan_topics", scan_topics_)) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load scan_topics. Set to empty.");
    }
    if (!private_nh_.getParam("pointcloud2_topics", cloud_topics_)) {
        ROS_WARN_STREAM("[LIDAR FUSION] Did not load pointcloud2_topics. Set to empty.");
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

    private_nh_.param("fused_scan_options/angle_increment", angle_increment_, -1.0);
    private_nh_.param("fused_scan_options/range_max", range_max_, -1.0);
    private_nh_.param("fused_scan_options/range_min", range_min_, -1.0);
    private_nh_.param("fused_scan_options/scan_time", scan_time_, -1.0);
    private_nh_.param("angle_min", angle_min_,-M_PI);
    private_nh_.param("angle_max", angle_max_,M_PI);

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

    num_lidars_ = scan_topics_.size();
    num_depth_sensors_ = cloud_topics_.size();

    //subscribe to scan msgs
    if(num_lidars_){
      std::string scan_topic_name;
      for(int i=0; i < max_topic_num_; i++){
        if(num_lidars_ > i)
          scan_topic_name = scan_topics_[i];
        else
          scan_topic_name = scan_topics_[num_lidars_-1];
        scan_sub_[i].subscribe(nh_, scan_topic_name, 20);
      }
    }
    //subscribe to cloud msgs
    if(num_depth_sensors_){
      std::string cloud_topic_name;
      for(int i=0; i < max_topic_num_; i++){
        if(num_depth_sensors_ > i)
          cloud_topic_name = cloud_topics_[i];
        else
          cloud_topic_name = cloud_topics_[num_depth_sensors_-1];
        cloud_sub_[i].subscribe(nh_, cloud_topic_name, 20);
      }
    }

    //initialize a laserscan message with default parameters (for PointCloud to laserscan conversion and no parameters are specified)
    scan_default_.angle_increment = M_PI/360;
    scan_default_.range_max = 4.0;
    scan_default_.range_min = 0.45;
    scan_default_.scan_time = 1.0/3;

    //register approximate_time callback according to sensors
    if(num_lidars_ > 0 && num_depth_sensors_ > 0)
      scan_pointcloud_sync_.registerCallback(boost::bind(&FusedScan::fusedScanCloudCallback, this, _1, _2, _3, _4,
                                                                                                    _5, _6, _7, _8));
    else if(num_depth_sensors_>0)
      pointcloud_sync_.registerCallback(boost::bind(&FusedScan::fusedCloudCallback, this, _1, _2, _3, _4));
    else
      scan_sync_.registerCallback(boost::bind(&FusedScan::fusedScanCallback, this, _1, _2, _3, _4));


    fused_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan> (fused_scan_topic_name_, 20);
    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped> (polygon_topic_name_, 5);
    fused_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (fused_pointcloud_topic_name_, 20);

}

void FusedScan::initializeParams(const sensor_msgs::LaserScan::ConstPtr& scan_front)
{
  if(angle_increment_ < 0)
    angle_increment_ = scan_front->angle_increment;

  if(range_max_ < 0)
    range_max_ = scan_front->range_max;

  if(range_min_ < 0)
    range_min_ = scan_front->range_min;

  if(scan_time_ < 0)
    scan_time_ = scan_front->scan_time;

  range_size_ = std::ceil((2 * M_PI)/angle_increment_);

  scan_fuse_.ranges.resize(range_size_, range_max_);
  scan_fuse_.intensities.resize(range_size_, 100.0);

  init_params_ = true;


}

//callback function for only laserscan fusion
void FusedScan::fusedScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_1, const sensor_msgs::LaserScan::ConstPtr& scan_2,
                                    const sensor_msgs::LaserScan::ConstPtr& scan_3, const sensor_msgs::LaserScan::ConstPtr& scan_4)
{
    if(!init_params_)
      initializeParams(scan_1);

    // scan_msg_.clear();
    // scan_msg_.insert(scan_msg_.end(),{*scan_1, *scan_2, *scan_3, *scan_4});
    scan_msg_ = {*scan_1, *scan_2, *scan_3, *scan_4};

    // std::cout << scan_msg_[0].intensities.size() << '\n';

    for(int i =0; i< num_lidars_; i++){
      if (!tflistener_.waitForTransform(scan_msg_[i].header.frame_id, base_link_,
          scan_msg_[i].header.stamp + ros::Duration().fromSec(scan_msg_[i].ranges.size()*scan_msg_[i].time_increment),
          ros::Duration(3.0))) {
          return;
      }
    }

    //convert scan to pointcloud
    cloud_fuse_.points.clear();
    sensor_msgs::PointCloud cloud_msg;
    for(int i=0; i<num_lidars_;i++){
      try
      {
        projector_.transformLaserScanToPointCloud(base_link_, scan_msg_[i], cloud_msg, tflistener_);
        //apply angle_filtering 
        convertPointCloudToPointCloud2(cloud_msg, cloud_conv_);
        processPointCloud(cloud_conv_, cloud_filtered_);
        cloud_fuse_.points.insert(cloud_fuse_.points.end(), cloud_filtered_.points.begin(), cloud_filtered_.points.end());
      }
      catch ( const tf2::TransformException& e )
      {
        ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
      }

    }

      cloud_fuse_.header.frame_id = cloud_msg.header.frame_id;
      cloud_fuse_.header.seq      = cloud_msg.header.seq;
      cloud_fuse_.header.stamp    = cloud_msg.header.stamp;

      //merge the point clouds
      if(angle_increment_ < 0)
        angle_increment_ = scan_1->angle_increment;
      mergePointClouds(angle_increment_);
      sendVisualization();

}


//callback function for only pointcloud fusion
void FusedScan::fusedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_1, const sensor_msgs::PointCloud2::ConstPtr& cloud_2,
                                      const sensor_msgs::PointCloud2::ConstPtr& cloud_3, const sensor_msgs::PointCloud2::ConstPtr& cloud_4)
{

  if(!init_params_){
    sensor_msgs::LaserScan::ConstPtr scan_ptr;
    scan_ptr.reset(new sensor_msgs::LaserScan(scan_default_));
    initializeParams(scan_ptr);
  }

  cloud2_msg_ = {*cloud_1, *cloud_2, *cloud_3, *cloud_4};


  for(int i =0; i< num_depth_sensors_; i++){
    if (!tflistener_.waitForTransform(cloud2_msg_[i].header.frame_id, base_link_,
        cloud2_msg_[i].header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }
  }

  //convert PointCloud2 to pointcloud
  sensor_msgs::PointCloud cloud_processed;
  cloud_fuse_.points.clear();
  for(int i=0; i<num_depth_sensors_;i++){
    try
    {
      processPointCloud(cloud2_msg_[i], cloud_processed);
      cloud_fuse_.points.insert(cloud_fuse_.points.end(), cloud_processed.points.begin(), cloud_processed.points.end());
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
      return;
    }

  }

    cloud_fuse_.header.frame_id = cloud_processed.header.frame_id;
    cloud_fuse_.header.seq      = cloud_processed.header.seq;
    cloud_fuse_.header.stamp    = cloud_processed.header.stamp;

    mergePointClouds(angle_increment_);
    sendVisualization();

}

//callback function for laser+PointCloud fusion
void FusedScan::fusedScanCloudCallback(const sensor_msgs::LaserScan::ConstPtr& scan_1, const sensor_msgs::LaserScan::ConstPtr& scan_2,
                                        const sensor_msgs::LaserScan::ConstPtr& scan_3, const sensor_msgs::LaserScan::ConstPtr& scan_4,
                                          const sensor_msgs::PointCloud2::ConstPtr& cloud_1, const sensor_msgs::PointCloud2::ConstPtr& cloud_2,
                                            const sensor_msgs::PointCloud2::ConstPtr& cloud_3, const sensor_msgs::PointCloud2::ConstPtr& cloud_4)
{

  if(!init_params_)
    initializeParams(scan_1);

  scan_msg_ = {*scan_1, *scan_2, *scan_3, *scan_4};
  cloud2_msg_ = {*cloud_1, *cloud_2, *cloud_3, *cloud_4};

  for(int i =0; i< num_lidars_; i++){
    if (!tflistener_.waitForTransform(scan_msg_[i].header.frame_id, base_link_,
        scan_msg_[i].header.stamp + ros::Duration().fromSec(scan_msg_[i].ranges.size()*scan_msg_[i].time_increment),
        ros::Duration(3.0))) {
        return;
    }
  }

  for(int i =0; i< num_depth_sensors_; i++){
    if (!tflistener_.waitForTransform(cloud2_msg_[i].header.frame_id, base_link_,
        cloud2_msg_[i].header.stamp + ros::Duration().fromSec(0.1),
        ros::Duration(3.0))) {
        return;
    }
  }
  //convert scan to pointcloud and PointCloud2 to PointCloud
  cloud_fuse_.points.clear();
  sensor_msgs::PointCloud cloud_msg, cloud_processed;

  for(int i=0; i<num_lidars_;i++){
    try
    {
      projector_.transformLaserScanToPointCloud(base_link_, scan_msg_[i], cloud_msg, tflistener_);
      cloud_fuse_.points.insert(cloud_fuse_.points.end(), cloud_msg.points.begin(), cloud_msg.points.end());
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
    }

  }
  for(int i=0; i<num_depth_sensors_;i++){
    try
    {
      processPointCloud(cloud2_msg_[i], cloud_processed);
      cloud_fuse_.points.insert(cloud_fuse_.points.end(), cloud_processed.points.begin(), cloud_processed.points.end());
    }
    catch ( const tf2::TransformException& e )
    {
      ROS_WARN_STREAM("[LIDAR FUSION] "<<e.what());
    }

  }

    cloud_fuse_.header.frame_id = cloud_msg.header.frame_id;
    cloud_fuse_.header.seq      = cloud_msg.header.seq;
    cloud_fuse_.header.stamp    = cloud_msg.header.stamp;

    //merge the point clouds
    mergePointClouds(angle_increment_);
    sendVisualization();


}

void FusedScan::mergePointClouds(double angle_increment) {

    cloud_crop_.header.frame_id = cloud_fuse_.header.frame_id;
    cloud_crop_.header.seq      = cloud_fuse_.header.seq;
    cloud_crop_.header.stamp    = cloud_fuse_.header.stamp;
    cloud_crop_.points.clear();

    std::fill(scan_fuse_.ranges.begin(), scan_fuse_.ranges.end(), range_max_);

    for (long int i = 0; i < cloud_fuse_.points.size(); i++){

        // check if point inside polygon
        Point cloud_point = Point(cloud_fuse_.points[i].x,cloud_fuse_.points[i].y);
        if(CGAL::bounded_side_2(polygon_.begin(), polygon_.end(), cloud_point, K()) == CGAL::ON_BOUNDED_SIDE){
          continue;
        }
        cloud_crop_.points.push_back(cloud_fuse_.points[i]);


        //calculate only if laser is in fusion
        if(angle_increment > 0){
          float angle = atan2f32(cloud_fuse_.points[i].y, cloud_fuse_.points[i].x);
          float range_val = hypotf32(cloud_fuse_.points[i].x, cloud_fuse_.points[i].y);
          int index = (int)((angle + M_PI)/angle_increment);
          if (range_val < scan_fuse_.ranges[index])
              scan_fuse_.ranges[index] = range_val;
          }

    }

}

void FusedScan::sendLaserVisualization() {
    scan_fuse_.header.frame_id = cloud_fuse_.header.frame_id;
    scan_fuse_.header.stamp    = cloud_fuse_.header.stamp;
    scan_fuse_.header.seq      = cloud_fuse_.header.seq;
    scan_fuse_.angle_increment = angle_increment_;
    scan_fuse_.angle_max       = angle_max_;
    scan_fuse_.angle_min       = angle_min_;
    scan_fuse_.range_max       = range_max_;
    scan_fuse_.range_min       = range_min_;
    scan_fuse_.scan_time       = scan_time_;
    scan_fuse_.time_increment  = scan_time_/range_size_;

    fused_scan_pub_.publish(scan_fuse_); //publish fused scan

}

void FusedScan::sendCloudVisualization() {

  //publish PointCloud2
  sensor_msgs::PointCloud2 fused_cloud;
  convertPointCloudToPointCloud2(cloud_crop_, fused_cloud);
  fused_pointcloud_pub_.publish(fused_cloud);

}

void FusedScan::sendPolygonVisualization(){
  // update polygon timestamp
  polygon_viz_.header.frame_id = base_link_;
  polygon_viz_.header.stamp = cloud_fuse_.header.stamp;
  polygon_pub_.publish(polygon_viz_); //publish polygon

}

void FusedScan::sendVisualization(){

  sendLaserVisualization();
  sendCloudVisualization();
  sendPolygonVisualization();

}

void FusedScan::processPointCloud(sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud& cloud_processed){

    sensor_msgs::LaserScan output;
    output.header = cloud_msg.header;
    output.header.frame_id = base_link_;

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;

    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.time_increment = scan_time_/ranges_size;


    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2 cloud2_msg_transformed;

    // Transform cloud if necessary
    if (!(output.header.frame_id == cloud_msg.header.frame_id))
    {
      try
      {
        pcl_ros::transformPointCloud (base_link_, cloud_msg, cloud2_msg_transformed, tflistener_);
        cloud_out.reset(new sensor_msgs::PointCloud2(cloud2_msg_transformed));
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      cloud_out.reset(new sensor_msgs::PointCloud2(cloud_msg));
    }

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
              iter_x != iter_x.end();
              ++iter_x, ++iter_y, ++iter_z)
    {

      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
      {
        ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)
      {
        ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_)
      {
        ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }

      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }

    }
    projector_.transformLaserScanToPointCloud(base_link_, output, cloud_processed, tflistener_);
  }


int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_fusion_node");
    ros::NodeHandle nh;
    FusedScan fs(nh);
    ros::spin();
}
