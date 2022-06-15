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

#include <selection_criteria/correct_odometry_kitti.h>


// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;



////////////////////////////////////////////////////////////////////////////////
// Will return points based on the conditions set by the algorithm
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
CorrectOdometry::CorrectOdometry (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;

  // Define the publishers
  odom_abs_pub = nh.advertise<nav_msgs::Odometry> ("/odom_abs", 3); // cannot do this, need to work on rotating the odometry not the pointcloud.
    

  // Create a ROS subscriber for computed odometry
  odom_sub_ = nh_.subscribe("/odom", 1, &CorrectOdometry::odom_callback, this);
  // odom_sub_ = nh_.subscribe("/odom_transformed", 1, &CorrectOdometry::odom_callback, this);


}


////////////////////////////////////////////////////////////////////////////////

void
CorrectOdometry::odom_callback(const nav_msgs::OdometryConstPtr& odom_in)
{

  nav_msgs::Odometry robot_odom;
  robot_odom = *odom_in;
  // Converting Odom Orientation from Quaternion to Roll Pitch and Yaw:

  double q_x = robot_odom.pose.pose.orientation.x;
  double q_y = robot_odom.pose.pose.orientation.y;
  double q_z = robot_odom.pose.pose.orientation.z;
  double q_w = robot_odom.pose.pose.orientation.w;

  // g_roll = atan2(2.0*(q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z);
  // g_pitch = asin(-2.0*(q_x*q_z - q_w*q_y));
  // g_yaw = atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z);

  // Quaternion
  tf2::Quaternion q(q_x, q_y, q_z, q_w);
  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);
  // Roll Pitch and Yaw from rotation matrix
  m.getRPY(g_roll, g_pitch, g_yaw);
  
  // g_roll = g_roll * (180.0 / M_PI);
  // g_pitch = g_pitch * (180.0 / M_PI);
  // g_yaw = g_yaw * (180.0 / M_PI); 

  //Because while finding error it does not make sense to have one at 180 and another at -180 becuse of the angle change fluctuation:
  //Also because after transformation, my yaw is pitch and therefore it is easier to compare the orientation without the transformation due to the adjustments i have to make.
  //Try to set yaw and roll to 0 afterwards and check if i can find same effect.
  //If i have further problems try to make changes in evo to only find absolute error
  g_roll = 0;
  g_pitch = - g_yaw;
  g_yaw = 0; 

  tf2::Quaternion newQuaternion;
  newQuaternion.setRPY( g_roll, g_pitch, g_yaw );  // Create this quaternion from roll/pitch/yaw (in radians)

  // Print the quaternion components (0,0,0,1)
  // ROS_INFO_STREAM("x: " << newQuaternion.getX() << " y: " << newQuaternion.getY() << 
  //                 " z: " << newQuaternion.getZ() << " w: " << newQuaternion.getW());
  robot_odom.pose.pose.orientation.x = newQuaternion.getX();
  robot_odom.pose.pose.orientation.y = newQuaternion.getY();
  robot_odom.pose.pose.orientation.z = newQuaternion.getZ();
  robot_odom.pose.pose.orientation.w = newQuaternion.getW();

  odom_abs_pub.publish(robot_odom);


  // Need to read gt data by syncronising the topics in the same callback, correct the angle, and convert it back to quaternion, and then correct the odometry
  // As Roll is the Yaw in my frame
  // if( (g_roll_gt < -10) && (g_yaw > 0) ) g_yaw -= 360.0;
  // if( (g_roll_gt > 10) && (g_yaw < 0) ) g_yaw += 360.0;

  std::cout<< g_roll <<std::endl;
  std::cout<< g_pitch <<std::endl;
  std::cout<< g_yaw <<std::endl;

  g_rpy.push_back(to_string(g_roll));
  g_rpy.push_back(to_string(g_pitch));
  g_rpy.push_back(to_string(g_yaw));

	// Appending the Values of Roll, Pitch and Yaw to a text file.
	// std::ofstream output_file("~/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/RPY.txt");
	std::ofstream output_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/RPY.txt");
	std::ostream_iterator<std::string> output_iterator(output_file, ",");
	std::copy(g_rpy.begin(), g_rpy.end(), output_iterator);





}


////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init () before using any other
    * part of the ROS system.
    */

  ros::init (argc, argv, "selection_criteria_localization");

  /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");

  // Create a selection criteria localization object
  CorrectOdometry SCLObject (nh);
  
   
  ros::spin ();
  
  return (0);

} 