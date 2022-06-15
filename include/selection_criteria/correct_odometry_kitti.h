/* MIT License

Copyright (c) 2022 Ahmed Adamjee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef CORRECT_ODOMETRY_KITTI_H_
#define CORRECT_ODOMETRY_KITTI_H_

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>

// ROS includes
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>


// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// MESSAGE FILTERS
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <cstdlib>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// TF2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>


// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>

// NAMESPACES
using namespace std;
using namespace message_filters;

// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;


/** \brief Correct Odometry Kitti Script
  *
  * \author Ahmed Adamjee
  */

class CorrectOdometry
{
  public:

    /** \brief  Class constructor. 
      *
      * \input[in] nh ROS node handle
      */
    CorrectOdometry(ros::NodeHandle& nh);


    /** \brief Odometry CallBack function.
      * 
      * \input[in] odom_in a Odometry nav_msgs const pointer
      */
    void
    odom_callback(const nav_msgs::OdometryConstPtr& odom_in);

      
    /* Variables */

    /** \brief Node Handler. */
    ros::NodeHandle nh_; 

    /** \brief ROS publishers. */
    ros::Publisher odom_abs_pub;

    /** \brief ROS subscribers. */
    ros::Subscriber odom_sub_;
    

    /** \brief Variable to save the Robot Odometry's Roll value for each pose */
    double g_roll;
    /** \brief Variable to save the Robot Odometry's Pitch value for each pose */
    double g_pitch;
    /** \brief Variable to save the Robot Odometry's Yaw value for each pose */
    double g_yaw;
    /** \brief List to save the Robot Odometry's Roll, Pitch and Yaw value in each iteration */
    std::vector<std::string> g_rpy;





  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif