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

#ifndef SELECTION_CRITERIA_LOCALIZATION_H_
#define SELECTION_CRITERIA_LOCALIZATION_H_

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <stdlib.h>
#include <cmath>
#include <iostream>

// ROS includes
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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


/** \brief Selection Criteria Localization Script
  *
  * \author Ahmed Adamjee
  */

class SCLocalization
{
  public:

    /** \brief  Class constructor. 
      *
      * \input[in] nh ROS node handle
      */
    SCLocalization(ros::NodeHandle& nh);

    /** \brief Cylinder Filter Condition 
      *
      * used for picking and placing at given position 
      * 
      * \input[in] x: x co-ordinate of point in point cloud
      * \input[in] y: y co-ordinate of point in point cloud
      * \input[in] z: z co-ordinate of point in point cloud
      * \input[in] cylinder_x_axis_origin: The origin of the cylinder in the x axis
      * \input[in] cylinder_radius: The radius of the cylinder
      * \input[in] cylinder_height: The height of the cylinder
      *  
      * \return true if condition is fulfilled
      */
    bool 
    cylinderCondition(double x,
                      double y,
                      double z,
                      double cylinder_x_axis_origin,
                      double cylinder_radius,
                      double cylinder_height);

    /** \brief Filter 
      *
      * Selection Criteria Filter
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      *  
      */
    void 
    Filter (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);

    /** \brief Cylinder Filter 
      *
      * Will return points outside the defined Cylinder
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[in] cylinder_x_axis_origin: The origin of the cylinder in the x axis
      * \input[in] cylinder_radius: The radius of the cylinder
      * \input[in] cylinder_height: The height of the cylinder
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      *  
      */
    void 
    cylinderFilter (PointCPtr &in_cloud_ptr,
                    PointCPtr &out_cloud_ptr,
                    double cylinder_x_axis_origin,
                    double cylinder_radius,
                    double cylinder_height);

    /** \brief Box Filter 
      *
      * Will return points inbetween the defined ceiling and floor of the Z axes
      * and also between the min0/max0 OR min1/max1 pairs on the X and Y axes
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[in] x_axis_min: Min filter bound in the x axis
      * \input[in] y_axis_min: Min filter bound in the y axis
      * \input[in] z_axis_min: Min filter bound in the z axis
      * \input[in] x_axis_max: Max filter bound in the x axis
      * \input[in] y_axis_max: Max filter bound in the y axis
      * \input[in] z_axis_max: Max filter bound in the z axis
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      *  
      */
    void 
    boxFilter (PointCPtr &in_cloud_ptr,
               PointCPtr &out_cloud_ptr,
               double x_axis_min, double x_axis_max,
               double y_axis_min, double y_axis_max,
               double z_axis_min, double z_axis_max);

    /** \brief Add 
      *
      * Adds point clouds
      * 
      * \input[in] cloud1 the input PointCloud2 pointer
      * \input[out] cloud2 the output PointCloud2 pointer with appeded points
      *  
      */
    void 
    Add (PointCPtr &cloud1,
             PointCPtr &cloud2);


    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_msg1 a PointCloud2 sensor_msgs const pointer
      */
    void
    callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1);


      
    /* Variables */

    /** \brief Node Handler. */
    ros::NodeHandle nh_; 

    /** \brief ROS publishers. */
    ros::Publisher pub_;

    /** \brief ROS subscribers. */
    ros::Subscriber sub_;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered;





  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif