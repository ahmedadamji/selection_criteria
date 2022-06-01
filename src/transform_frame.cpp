
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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define RAD2DEG 57.295779513


// ros::Publisher pub;
ros::Publisher odom_pub;


// void callback (const nav_msgs::OdometryConstPtr& odom_in) // check how to do tf transform for odometry, and if there is a pointer object for odometry
// void callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in)
void callback (nav_msgs::Odometry odom_in)
{
ros::Duration cache_(1);
tf2_ros::Buffer tfBuffer(cache_);
tf2_ros::TransformListener tfListener(tfBuffer);
// sensor_msgs::PointCloud2 cloud_out;
nav_msgs::Odometry odom_out;

    
geometry_msgs::TransformStamped transformStamped;

try {
    transformStamped = tfBuffer.lookupTransform("base_link", "velo_link",ros::Time(0), ros::Duration(.50));
    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "velo_link";
    // //transformStamped.child_frame_id = ;
    // // transformStamped.transform.translation.x = msg->x;
    // // transformStamped.transform.translation.y = msg->y;
    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    // tf2::Quaternion q;
    // // q.setRPY(0, 0, msg->theta);
    // // A yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis.
    // // A pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis.
    // // A roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis.
    // q.setRPY(0, - M_PI_2, - M_PI_2);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();
    // tf2::doTransform(*cloud_in, cloud_out, transformStamped); //what is the correct command if this is odometry
    geometry_msgs::Pose pose_in = odom_in.pose.pose;
    geometry_msgs::Pose pose_out;
    tf2::doTransform(pose_in, pose_out, transformStamped); //what is the correct command if this is odometry
    odom_out = odom_in;
    odom_out.pose.pose = pose_out;
}
catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    //ros::Duration(1.0).sleep();
}

// pub.publish(cloud_out);
odom_pub.publish(odom_out);
}

int main (int argc, char** argv){

    ros::init (argc, argv, "getin_local_frame");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/kitti/velo/pointcloud", 1, callback);
    ros::Subscriber sub = nh.subscribe("/odom", 10, callback);


    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("/kitti/velo/pointcloud_transformed", 1); // cannot do this, need to work on rotating the odometry not the pointcloud.
    odom_pub = nh.advertise<nav_msgs::Odometry> ("/odom_transformed", 1); // cannot do this, need to work on rotating the odometry not the pointcloud.
    
 
    ros::spin();
    
    return 0;
} 
