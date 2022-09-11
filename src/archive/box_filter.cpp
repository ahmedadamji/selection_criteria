
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>

#define __APP_NAME__ "box_filter"

// variables that define an inner and outer box for filtering points beyond the region of interest and within the truck itself
static double x_axis_min_, y_axis_min_, x_axis_max_, y_axis_max_,
	      z_axis_min_, z_axis_max_;

ros::Publisher _pub_box_filtered_cloud;
std_msgs::Header _velodyne_header;

std::string _box_filtered_topic, _input_topic;

// Box filter
// Will return points inbetween the defined ceiling and floor of the Z axes
// and also between the min0/max0 OR min1/max1 pairs on the X and Y axes
void boxFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
		     double z_axis_min = -2.5, double z_axis_max = 0,
		     double x_axis_min = -20, double x_axis_max = 25,
		     double y_axis_min = -20, double y_axis_max = 25)
{
  out_cloud_ptr->points.clear();
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    if ( it->z >= z_axis_min && it->z <= z_axis_max && // within the Z limits
         it->x >= x_axis_min && it->x <= x_axis_max && it->y >= y_axis_min && it->y <= y_axis_max ) // Not within the central limits
    {
       out_cloud_ptr->points.push_back(*it);
    } 
  } 
}

void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
  // _start = std::chrono::system_clock::now();


    pcl::PointCloud<pcl::PointXYZI>::Ptr unclipped_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr box_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);
    
    _velodyne_header = in_sensor_cloud->header;
    pcl::fromROSMsg(*in_sensor_cloud, *unclipped_sensor_cloud_ptr);


    boxFilter(unclipped_sensor_cloud_ptr, box_filtered_points,
		      z_axis_min_, z_axis_max_,
		      x_axis_min_, x_axis_max_,
		      y_axis_min_, y_axis_max_);
    
    publishCloud(&_pub_box_filtered_cloud, box_filtered_points);

}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "box_filter_init");

  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");

  private_nh.param("/box_filter/input_topic", _input_topic, std::string("/points_raw"));
  ROS_INFO("[%s] Input topic: %s", __APP_NAME__, _input_topic.c_str());
  private_nh.param("/box_filter/output_topic", _box_filtered_topic, std::string("/box_filtered_points_raw"));
  ROS_INFO("[%s] Input topic: %s", __APP_NAME__, _box_filtered_topic.c_str());

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber pc_sub = h.subscribe(_input_topic, 1, velodyne_callback); 

  // Create ROS publisher for box filtered point cloud
  _pub_box_filtered_cloud = h.advertise<sensor_msgs::PointCloud2>(_box_filtered_topic, 1);

  // box clip params
  private_nh.param("/box_filter/x_axis_min", x_axis_min_, -15.0);
  ROS_INFO("[%s] x_axis_min: %f", __APP_NAME__, x_axis_min_);
  private_nh.param("/box_filter/x_axis_max", x_axis_max_, 15.0);
  ROS_INFO("[%s] x_axis_max: %f", __APP_NAME__, x_axis_max_);
  private_nh.param("/box_filter/y_axis_min", y_axis_min_, -20.0);
  ROS_INFO("[%s] y_axis_min: %f", __APP_NAME__, y_axis_min_);
  private_nh.param("/box_filter/y_axis_max", y_axis_max_, 60.0);
  ROS_INFO("[%s] y_axis_max: %f", __APP_NAME__, y_axis_max_);
  private_nh.param("/box_filter/z_axis_min", z_axis_min_, -2.5);
  ROS_INFO("[%s] z_axis_min: %f", __APP_NAME__, z_axis_min_);
  private_nh.param("/box_filter/z_axis_max", z_axis_max_, -1.0);
  ROS_INFO("[%s] z_axis_max: %f", __APP_NAME__, z_axis_max_);
  
  

  // Spin
  ros::spin();

}
