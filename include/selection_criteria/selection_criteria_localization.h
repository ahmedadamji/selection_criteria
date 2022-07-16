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


// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
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
#include <sensor_msgs/Imu.h>

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


// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// TF2 specific includes
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
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>
#include <cstdlib>

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
      * used for filtering points inside a defined cylinder
      * 
      * \input[in] x: x co-ordinate of point in point cloud
      * \input[in] y: y co-ordinate of point in point cloud
      * \input[in] z: z co-ordinate of point in point cloud
      * \input[in] x_axis_origin: The origin of the cylinder in the x axis
      * \input[in] radius: The radius of the cylinder
      * \input[in] height: The height of the cylinder
      *  
      * \return true if condition is fulfilled
      */
    bool 
    cylinderCondition(double x,
                      double y,
                      double z,
                      float x_axis_origin,
                      float radius,
                      float height);

      /** \brief Radius Filter Condition 
      *
      * used for filtering points inside a defined radial region
      * 
      * \input[in] x: x co-ordinate of point in point cloud
      * \input[in] y: y co-ordinate of point in point cloud
      * \input[in] z: z co-ordinate of point in point cloud
      * \input[in] min_radius: The minimum filtering radius
      * \input[in] max_radius: The maximum filtering radius
      *  
      * \return true if condition is fulfilled
      */
    bool 
    radiusCondition(double x,
                    double y,
                    double z,
                    float min_radius,
                    float max_radius);

    /** \brief Ring Filter Condition 
      *
      * used for filtering points outside a defined ring
      * 
      * \input[in] x: x co-ordinate of point in point cloud
      * \input[in] y: y co-ordinate of point in point cloud
      * \input[in] z: z co-ordinate of point in point cloud
      * \input[in] x_axis_origin: The origin of the ring in the x axis
      * \input[in] ring_min_radius: The min radius of the ring
      * \input[in] ring_max_radius: The max radius of the ring
      * \input[in] ring_height: The height of the ring
      *  
      * \return true if condition is fulfilled
      */
    bool 
    ringCondition(double x,
                  double y,
                  double z,
                  float x_axis_origin,
                  float ring_min_radius,
                  float ring_max_radius,
                  float ring_height);

    /** \brief Floor Filter Condition 
      *
      * used for filtering points that lie on the floor based on a simple height threshold
      * 
      * \input[in] filter_floor: a bool indicating weather the floor should be filtered
      *  
      * \return true if condition is fulfilled
      */
    bool 
    floorFilter (bool filter_floor);

    /** \brief Update ROS Params
      *
      * Updates the values stored in ROS parameters to the appropriate values
      * 
      */
    void 
    updateROSParams ();

    /** \brief Compute FilteredPointsData 
      *
      * Computes FilteredPointsData of the points filtered from chosen criteria and displays and stores them to a text file
      * 
      */
    void 
    computeFilteredPointsData ();

    /** \brief Transform Robot Coordinates 
      *
      * Transforms point coordinates of the robot from the lidar to the world coordinate frame
      * 
      */
    void 
    transformRobotCoordinates ();

    /** \brief Compute Trajectory Information
      *
      * Compute additional robot trajectory information to odometry
      * 
      */
    void 
    computeTrajectoryInformation ();

    /** \brief Transform Point Coordinates 
      *
      * Transforms point coordinates of the feature from the lidar to the world coordinate frame
      * 
      */
    void 
    transformPointCoordinates ();

    /** \brief Compute Angle Deviation
      *
      * Computes the angle deviation of a point relative to previous frame
      * 
      * \output angle The angle deviation of a point relative to previous frame
      */
    double
    computeAngleDeviation ();

    /** \brief Compute Angle Deviation Statistics
      *
      * Computes the statistics of the cpmputed angle deviation of all the points relative to previous frame
      * 
      */
    void
    computeAngleDeviationStatistics ();


    /** \brief Compute Observation Angle
      *
      * Computes the angle at which a point was observed from the lidar, relative to the parallel axis.
      * 
      * \output angle The observation angle of a point
      */
    double
    computeObservationAngle();


    /** \brief Filter 
      *
      * Selection Criteria Filter
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    Filter (PointCPtr &in_cloud_ptr,
            PointCPtr &out_cloud_ptr,
            PointCPtr &vis_cloud_ptr);

    /** \brief Cylinder Filter 
      *
      * Will return points outside the defined Cylinder
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[in] x_axis_origin: The origin of the cylinder in the x axis
      * \input[in] radius: The radius of the cylinder
      * \input[in] height: The height of the cylinder
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    cylinderFilter (PointCPtr &in_cloud_ptr,
                    PointCPtr &out_cloud_ptr,
                    PointCPtr &vis_cloud_ptr,
                    float x_axis_origin,
                    float radius,
                    float height);

    /** \brief Radius Filter 
      *
      * Will return points within the defined radius bounds
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[in] min_radius: The minimum filtering radius
      * \input[in] max_radius: The maximum filtering radius
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    radiusFilter (PointCPtr &in_cloud_ptr,
                  PointCPtr &out_cloud_ptr,
                  PointCPtr &vis_cloud_ptr,
                  float min_radius,
                  float max_radius);
    
      /** \brief Ring Filter 
      *
      * Will return points inside the defined Ring
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[in] x_axis_origin: The origin of the ring in the x axis
      * \input[in] ring_min_radius: The min radius of the ring
      * \input[in] ring_max_radius: The max radius of the ring
      * \input[in] ring_height: The height of the ring
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    ringFilter (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr,
                PointCPtr &vis_cloud_ptr,
                float x_axis_origin,
                float ring_min_radius,
                float ring_max_radius,
                float ring_height);

    /** \brief Box Filter 
      *
      * Will return points inbetween the defined ceiling and floor of the Z axes
      * and also between the min0/max0 OR min1/max1 pairs on the X and Y axes
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[in] x_axis_min: Min filter bound in the x axis
      * \input[in] y_axis_min: Min filter bound in the y axis
      * \input[in] z_axis_min: Min filter bound in the z axis
      * \input[in] x_axis_max: Max filter bound in the x axis
      * \input[in] y_axis_max: Max filter bound in the y axis
      * \input[in] z_axis_max: Max filter bound in the z axis
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    boxFilter (PointCPtr &in_cloud_ptr,
               PointCPtr &out_cloud_ptr,
               PointCPtr &vis_cloud_ptr,
               float x_axis_min, float x_axis_max,
               float y_axis_min, float y_axis_max,
               float z_axis_min, float z_axis_max);


    /** \brief Angle Deviation Filter 
      *
      * Will return points that have moved within a certain angle threshold
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[in] min_angle: The min angle threshold by which a point has been predicted to move by
      * \input[in] max_angle: The max angle threshold by which a point has been predicted to move by
      * 
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      * \input[out] vis_cloud_ptr the vis PointCloud2 pointer, to visualize selected points in rviz clearly
      *  
      */
    void 
    angleDeviationFilter (PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr,
                          PointCPtr &vis_cloud_ptr,
                          float min_angle,
                          float max_angle);

    /** \brief Add 
      *
      * Adds point clouds
      * 
      * \input[in] cloud1 the input PointCloud2 pointer
      * 
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

    /** \brief Get Robot Velocity 
      *
      * used to get the vehicle velocity based on Odometry
      * 
      * \input[in] robot_twist a Twist geometry_msgs const pointer
      * 
      * \return velocity
      */
    double
    getVelocity(const geometry_msgs::Twist& robot_twist);

    /** \brief Odometry CallBack function.
      * 
      * \input[in] odom_in a Odometry nav_msgs const pointer
      */
    void
    odom_callback(const nav_msgs::OdometryConstPtr& odom_in);

    /** \brief IMU CallBack function.
      * 
      * \input[in] IMU_in a IMU sensor_msgs const pointer
      */
    void
    imu_callback(const sensor_msgs::Imu::ConstPtr& imu_in);

      
    /* Variables */

    /** \brief Node Handler. */
    ros::NodeHandle nh_; 

    /** \brief ROS publishers. */
    ros::Publisher pub_;
    ros::Publisher pub_vis_selected_points_;
    ros::Publisher odom_abs_pub;
    ros::Publisher pub_angle_deviation_mean_;
    ros::Publisher pub_angle_deviation_std_;
    ros::Publisher pub_angle_deviation_min_;
    ros::Publisher pub_angle_deviation_max_;

    /** \brief ROS subscribers. */
    ros::Subscriber sub_;
    ros::Subscriber floor_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;

    /** \brief ROS Time in seconds. */
    double g_current_time;

    /** \brief Previous ROS Time in seconds. */
    double g_previous_time;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered;

    /** \brief Size of Input Point Cloud. */
    int g_in_cloud_size;

    /** \brief Size of Out Point Cloud. */
    int g_out_cloud_size;
    
    /** \brief Total Size of Input Point Cloud. */
    int g_total_input_points;

    /** \brief Total Size of Output Point Cloud. */
    int g_total_output_points;

    /** \brief Total Size of Filtered Points. */
    int g_total_filtered_points;

    /** \brief Number of recorded Point Cloud Frames. */
    int g_total_number_of_frames;

    /** \brief Average Size of Input Point Cloud. */
    double g_average_input_points;

    /** \brief Average Size of Output Point Cloud. */
    double g_average_output_points;

    /** \brief Average Size of Filtered Points. */
    double g_average_filtered_points;

    /** \brief X-Coordinate of curent point in the pointcloud. */
    double g_x;

    /** \brief Y-Coordinate of curent point in the pointcloud. */
    double g_y;

    /** \brief Z-Coordinate of curent point in the pointcloud. */
    double g_z;

    /** \brief Name of Evaluation Dataset. */
    string g_dataset;

    /** \brief Name of Evaluation Sequence. */
    string g_sequence;

    /** \brief Name of used Filter. */
    string g_filter_name = "";

    /** \brief Name of File to save statistics to. */
    string g_file_name;

    /** \brief Bool to determine weather to filter the floor. */
    bool g_filter_floor = true;


    /** \brief TF listener definition. */
    tf::TransformListener g_listener_;

    /** \brief Robot coordinate in lidar frame. */
    geometry_msgs::PointStamped g_robot_lidar_frame_coordinate;

    /** \brief Robot coordinate in world frame. */
    geometry_msgs::PointStamped g_robot_world_frame_coordinate;

    /** \brief Robot coordinate in world frame in the previous position. */
    geometry_msgs::PointStamped g_previous_robot_world_frame_coordinate;

    /** \brief Current point coordinate in lidar frame. */
    geometry_msgs::PointStamped g_point_lidar_frame_coordinate;

    /** \brief Current point coordinate in world frame. */
    geometry_msgs::PointStamped g_point_world_frame_coordinate;


    /** \brief Current IMU reading from imu sensor. */
    sensor_msgs::Imu g_robot_imu;

    /** \brief Current orientation reading from imu sensor. */
    geometry_msgs::Quaternion g_robot_orientation;

    // /** \brief Current angular velocity reading from imu sensor. */
    // geometry_msgs::Vector3 g_robot_angular_velocity;
    // /** \brief Current angular velocity reading in x from imu sensor. */
    // double g_robot_angular_velocity_x;
    // /** \brief Current angular velocity reading in y from imu sensor. */
    // double g_robot_angular_velocity_y;
    // /** \brief Current angular velocity reading in z from imu sensor. */
    // double g_robot_angular_velocity_z;
    // /** \brief Current absolute angular velocity reading from imu. */
    // double g_robot_angular_velocity_abs;

    /** \brief Current linear acceleration reading from imu sensor. */
    geometry_msgs::Vector3 g_robot_linear_acceleration;
    /** \brief Current linear acceleration reading in x from imu sensor. */
    double g_robot_linear_acceleration_x;
    /** \brief Current linear acceleration reading in y from imu sensor. */
    double g_robot_linear_acceleration_y;
    /** \brief Current linear acceleration reading in z from imu sensor. */
    double g_robot_linear_acceleration_z;
    // /** \brief Current absolute linear acceleration reading in x from imu sensor. */
    // double g_robot_linear_acceleration_abs;

    /** \brief Current linear velocity computed in x. */
    double g_robot_linear_velocity_x;
    /** \brief Current linear velocity computed in y. */
    double g_robot_linear_velocity_y;
    /** \brief Current linear velocity computed in z. */
    double g_robot_linear_velocity_z;
    /** \brief Current linear velocity computed. */
    double g_robot_linear_velocity_abs;
    
    /** \brief Current previous linear velocity computed in x. */
    double g_robot_previous_linear_velocity_x;
    /** \brief Current previous linear velocity computed in y. */
    double g_robot_previous_linear_velocity_y;
    /** \brief Current previous linear velocity computed in z. */
    double g_robot_previous_linear_velocity_z;
    // /** \brief Previous linear velocity computed. */
    // double g_robot_previous_linear_velocity_abs;

    /** \brief Current angular velocity computed. */
    double g_robot_angular_velocity;
    /** \brief Current Robot angle. */
    double g_robot_angle;
    /** \brief Previous Robot angle. */
    double g_robot_previous_angle;




    // Parameters to compute the angle deviation of points with respect to the previous frame:
    /** \brief The vector to store the vector the robot has translated with respect to its previous frame */
    std::vector<double> g_vdt{0.0, 0.0, 0.0};
    /** \brief The vector to store the vector from the robots previous position to the observed point */
    std::vector<double> g_d1{0.0, 0.0, 0.0};
    /** \brief The vector to store the vector from the robots current position to the observed point */
    std::vector<double> g_d2{0.0, 0.0, 0.0};
    /** \brief modulus of vdt */
    double g_mod_vdt;
    /** \brief modulus of d1 */
    double g_mod_d1;
    /** \brief modulus of d2 */
    double g_mod_d2;
    /** \brief square of modulus of d1 */
    double g_mod_d1_sqr;
    /** \brief square of modulus of d2 */
    double g_mod_d2_sqr;


    //The vector to store the angle deviation of all points in a vector for visualization
    std::vector<double> angle_deviation_vec;
    //The std msg to store the statistics of angle deviation of all points for visualization
    std_msgs::Float32 angle_deviation_mean;
    std_msgs::Float32 angle_deviation_std;
    std_msgs::Float32 angle_deviation_min;
    std_msgs::Float32 angle_deviation_max;
    double previous_angle_deviation_mean;
    double previous_angle_deviation_std;
    double previous_angle_deviation_min;
    double previous_angle_deviation_max;

    /** \brief number of points that satisfied the constraints of the angle deviation filter */
    int g_matched_angle_deviation;



    // Additional parameters to compute the observation angle of points:
    /** \brief The vector to store the vector from the robots current position to where the observed point projects onto lidar's plane*/
    std::vector<double> g_do{0.0, 0.0, 0.0};
    /** \brief The vector to store the vector from where the observed point projects onto lidar's plane to the observed point */
    std::vector<double> g_dp{0.0, 0.0, 0.0};
    /** \brief modulus of do */
    double g_mod_do;
    /** \brief modulus of dp */
    double g_mod_dp;







  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif