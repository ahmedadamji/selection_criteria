
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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define RAD2DEG 57.295779513

// REFERENCE: https://towardsdatascience.com/kitti-coordinate-transformations-125094cd42fb


// ros::Publisher pub;
ros::Publisher odom_pub;


// NAMESPACES
using namespace std;


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
nav_msgs::Odometry odom_out;


geometry_msgs::TransformStamped transformStamped;

try {
    transformStamped = tfBuffer.lookupTransform("camera_color_left", "velo_link",ros::Time(0), ros::Duration(5));
    // check based on results weather we need the translation
    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    geometry_msgs::Pose pose_in = odom.pose.pose;
    geometry_msgs::Pose pose_out;
    tf2::doTransform(pose_in, pose_out, transformStamped);
    odom_out = odom;
    odom_out.pose.pose = pose_out;


    odom_out.pose.pose.orientation = odom.pose.pose.orientation;


    // Publishing Odometry
    odom_pub.publish(odom_out);
    
} catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    //ros::Duration(1.0).sleep();
}

}

int main (int argc, char** argv){

    ros::init (argc, argv, "getin_local_frame");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/kitti/velo/pointcloud", 1, callback);
    ros::Subscriber sub = nh.subscribe("/odom_abs", 3, callback);


    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("/kitti/velo/pointcloud_transformed", 1); // cannot do this, need to work on rotating the odometry not the pointcloud.
    odom_pub = nh.advertise<nav_msgs::Odometry> ("/odom_transformed", 3); // cannot do this, need to work on rotating the odometry not the pointcloud.
    
 
    ros::spin();
    
    return 0;
} 
