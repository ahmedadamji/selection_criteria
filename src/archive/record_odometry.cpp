
#define _USE_MATH_DEFINES

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cmath>
#include <iostream>
#include "std_msgs/String.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sstream>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <string>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/recorder.h>
// #include <bag_recorder/Rosbag.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define RAD2DEG 57.295779513

// REFERENCE: 



// NAMESPACES
using namespace std;


// ros::record::Recorder recorder;
// Recording Odometry to ROSBAG for Evaluation




// void callback (const nav_msgs::OdometryConstPtr& odom_in) // check how to do tf transform for odometry, and if there is a pointer object for odometry
// void callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in)
void callback (const nav_msgs::OdometryConstPtr& odom_in)
{

    ros::Duration cache_(5);
    // ros::Duration cache_ = genpy.Time(secs=pow(2,31)-1);
    tf2_ros::Buffer tfBuffer(cache_);
    tf2_ros::TransformListener tfListener(tfBuffer);
    // sensor_msgs::PointCloud2 cloud_out;
    nav_msgs::Odometry odom = *odom_in;

    string dataset;
    string sequence;
    string filter_name;
    string file_name;
    string file_path;
    int number_of_frames;

    ros::param::get("/dataset", dataset);
    ros::param::get("/sequence", sequence);
    ros::param::get("/filter_name", filter_name);
    ros::param::get("/number_of_frames", number_of_frames);

    // file_name = dataset + "_" + sequence + "_" + filter_name + ".bag";
    // file_path = "/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + dataset + "/" + sequence + "/results/trajectories/" + file_name;

    // rosbag::Bag bag;
    
    // if (number_of_frames == 1) {
    //     bag.open(file_path, rosbag::bagmode::Write);
    // }
    // else{
    //     bag.open(file_path, rosbag::bagmode::Append);
    // }
    // // bag.open(file_path, rosbag::bagmode::Write);
    // bag.write("/odom", ros::Time::now(), odom);
    // bag.close();
    
    // recorder.open(file_path);
    // recorder.record("/odom", odom, ros::Time::now());
    // recorder.close();



}

int main (int argc, char** argv){

    ros::init (argc, argv, "record");
    ros::NodeHandle nh;
    
    // rosbag::RecorderOptions options;
    // rosbag::Recorder recorder(options);
    // recorder.run();

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/kitti/velo/pointcloud", 1, callback);
    ros::Subscriber sub = nh.subscribe("/odom_transformed", 3, callback);

 
    ros::spin();
    
    return 0;
} 
