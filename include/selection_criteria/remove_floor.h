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

#ifndef REMOVE_FLOOR_H_
#define REMOVE_FLOOR_H_

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <sstream>


// ROS includes
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
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
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
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


/** \brief Remove Floor Script
  *
  * \author Ahmed Adamjee
  */

class RmFloor
{
  public:

    /** \brief  Class constructor. 
      *
      * \input[in] nh ROS node handle
      */
    RmFloor(ros::NodeHandle& nh);


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

    // FROM HDL_GRAPH_SLAM

    /** \brief Point Cloud Floor Plane Detection
     * 
     * \input[in] cloud1 a PointCloud2 sensor_msgs const pointer
     * 
     */
    void
    detect(PointCPtr &cloud1);

    /** \brief Plane Clip
     * 
     * \input[in] src_cloud a PointCloud2 sensor_msgs const pointer
     * \input[in] plane a ...
     * \input[in] negative a ...
     * \input[out] cloud2 the output PointCloud2 pointer with appeded points
     * 
     * 
     */
    void
    plane_clip(PointCPtr &src_cloud,
             PointCPtr &cloud2, const Eigen::Vector4f& plane, bool negative);

    /** \brief Detect points with non-vertical normals
     * 
     * \input[in] cloud a PointCloud2 sensor_msgs const pointer
     * \input[in] plane a ...
     * \input[out] cloud2 the output PointCloud2 pointer with appeded points
     * 
     */
    void
    normal_detection(PointCPtr& cloud,
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
    ros::Subscriber odom_sub_;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered;

    
    /** \brief X-Coordinate of curent point in the pointcloud. */
    double g_x;

    /** \brief Y-Coordinate of curent point in the pointcloud. */
    double g_y;

    /** \brief Z-Coordinate of curent point in the pointcloud. */
    double g_z;




    // Floor detection parameters from hdl_graph_slam:
    /** \brief Approximate sensor tilt angle [deg]. */
    double tilt_deg;
    /** \brief Approximate sensor height [m]. */
    double sensor_height;
    /** \brief Points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection. */
    double height_clip_range;
    /** \brief Minimum number of support points of RANSAC to accept a detected floor plane. */
    int floor_pts_thresh;
    /** \brief Verticality check thresold for the detected floor plane [deg]. */
    double floor_normal_thresh;
    /** \brief If true, points with "non-"vertical normals will be filtered before RANSAC. */
    bool use_normal_filtering;
    /** \brief "Non-"verticality check threshold [deg]. */
    double normal_filter_thresh;




  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif