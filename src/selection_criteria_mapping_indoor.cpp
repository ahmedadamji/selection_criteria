#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// NEW additions
#include<iostream>
#include<cmath>
using namespace std;
using namespace message_filters;

std_msgs::Header _output_header;

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
///////////////////////////////////
// Will return points based on the conditions set by the algorithm

void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{
  out_cloud_ptr->points.clear();
  
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    //out_cloud_ptr->points.push_back(*it);
    // if ( it->z >= 0.2)
    // {
    //   out_cloud_ptr->points.push_back(*it);
    // }

    // float max_radius = 0.2;
    // float min_radius = 0.02;
    // if ((( pow(it->x,2) + pow(it->y,2) ) >= pow(min_radius,2)) && (( pow(it->x,2) + pow(it->y,2) ) <= pow(max_radius,2)))
    // {
    //   out_cloud_ptr->points.push_back(*it);
    // }
    
    float max_radius = 0.05;
    if (( pow(it->x,2) + pow(it->y,2) ) <= pow(max_radius,2))
    {
      out_cloud_ptr->points.push_back(*it);
    }

    
  } 

}

// add point clouds
void Add(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2)
{
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud1->begin(); it != cloud1->end(); it++)
  {
    cloud2->points.push_back(*it);
  } 
  
}
//////////////////////
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/selected_points", 10);
  }
//  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1, const sensor_msgs::PointCloud2ConstPtr& cloud_msg2 )
// {// declare type
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1f(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);

// // These are the two synced clouds we are subscribing to, however both of these are currently same
// pcl::fromROSMsg(*cloud_msg1, *cloud1f);
// pcl::fromROSMsg(*cloud_msg2, *cloud2);
// //////////////////////////////////////////////////
// // add both clouds
// Filter(cloud1f, cloud_all); //removed suspected unneccesary points

// // // Previous condition -->
// // Filter(cloud1f, cloud1); //removed suspected unneccesary points
// // // Why do we need to add these clouds?
// // Add(cloud1, cloud2);// add rm floor
// // Add(cloud2, cloud_all);


// // write head and publish output
// _output_header = cloud_msg1->header;
//   sensor_msgs::PointCloud2 output;
   
//   pcl::toROSMsg(*cloud_all, output);
//   output.header = _output_header;
//   pub_.publish (output);
// }

 void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{// declare type
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1f(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);

// These are the two synced clouds we are subscribing to, however both of these are currently same
pcl::fromROSMsg(*cloud_msg1, *cloud1f);
//////////////////////////////////////////////////
// add both clouds
Filter(cloud1f, cloud_all); //removed suspected unneccesary points

// // Previous condition -->
// Filter(cloud1f, cloud1); //removed suspected unneccesary points
// // Why do we need to add these clouds?
// Add(cloud1, cloud2);// add rm floor
// Add(cloud2, cloud_all);


// write head and publish output
_output_header = cloud_msg1->header;
  sensor_msgs::PointCloud2 output;
   
  pcl::toROSMsg(*cloud_all, output);
  output.header = _output_header;
  pub_.publish (output);
}

 private:
  ros::NodeHandle nh_; 
  ros::Publisher pub_; 

};
////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  ros::init (argc, argv, "selection_criteria");
  ros::NodeHandle n;

  // Which topic am i supposed to subscribe and publish to for localization and mapping
  // this is normally done with different topics
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c1(n, "/filtered_points", 1); //check if anything is even published to filtered points
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c2(n, "/filtered_points", 1);
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), c1, c2);


  // SubscribeAndPublish SAPObject;
  
  // sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, &SAPObject, _1, _2));


  SubscribeAndPublish SAPObject;

  // Create a ROS subscriber for the input point cloud
  //nh.subscribe("/rs16_tc/rslidar_points", &SubscribeAndPublish::callback, &SAPObject, 1);
  // nh.subscribe("/rs16_tc/rslidar_points", 1000, boost::bind(&SubscribeAndPublish::callback, &SAPObject, _1));
  ros::Subscriber sub = nh.subscribe("/filtered_points", 1000, &SubscribeAndPublish::callback, &SAPObject);

  
   
  ros::spin ();
return 0;
} 
