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

#include <selection_criteria/selection_criteria_localization.h>


// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;



////////////////////////////////////////////////////////////////////////////////
// Will return points based on the conditions set by the algorithm
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
SCLocalization::SCLocalization (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  debug_ (false)
{
  nh_ = nh;

  // Define the publishers
  pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/selected_points", 3, true);
  
  // Initialize public variables
  // g_vg_leaf_sz = 0.01;
  // g_x_thrs_min = -0.7; 
  // g_x_thrs_max = -0.5;
  // g_y_thrs_min = 0.0;
  // g_y_thrs_max = 0.4;
  // g_cf_red = 25.5;
  // g_cf_blue = 204;
  // g_cf_green = 25.5;
  // g_k_nn = 50;


  // // Create a ROS subscriber for the input point cloud
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c1(nh_, "/filtered_points", 1); //check if anything is even published to filtered points
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c2(nh_, "/filtered_points", 1);
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), c1, c2);
  
  // sync.registerCallback(boost::bind(&SCLocalization::callback, this, _1, _2));


  // Create a ROS subscriber for the input point cloud
  // sub_ = nh_.subscribe("/filtered_points_without_floor", 3, &SCLocalization::callback, this);
  sub_ = nh_.subscribe("/filtered_points", 3, &SCLocalization::callback, this);
  
  // Create a ROS subscriber for computed odometry
  odom_sub_ = nh_.subscribe("/odom_transformed", 1, &SCLocalization::odom_callback, this);

}


////////////////////////////////////////////////////////////////////////////////

bool
SCLocalization::cylinderCondition(double x,
                                  double y,
                                  double z,
                                  double x_axis_origin = 0,
                                  double radius = 3,
                                  double height = 100)
{
  // Formula for the Volume of a cylinder: M_PI * (radius^2) * height
  // Formula for the Radius of a cylinder: sqrt(Volume / (M_PI * height))
  // Formula for the Height of a cylinder: Volume / (M_PI * (radius^2))
  // Formula for the Diameter of a cylinder: (sqrt(Volume / (M_PI * height)))/2

  // the cylinder is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
  if (!(( x >= (-x_axis_origin - height) && x <= (x_axis_origin + height)) && // within the Z limits
        (( pow(z,2) + pow(y,2) ) <= pow(radius,2)))) // within the radius limits
  {
    return true;
  }
  else
  {
    return false;
  }

}

////////////////////////////////////////////////////////////////////////////////

bool
SCLocalization::radiusCondition(double x,
                                double y,
                                double z,
                                double min_radius = 0,
                                double max_radius = 250)
{

  if ((( pow(x, 2) + pow(y, 2) ) >= pow(min_radius, 2)) && (( pow(x, 2) + pow(y, 2) ) <= pow(max_radius, 2))) // within the radius limits
  {
    return true;
  }
  else
  {
    return false;
  }

}
////////////////////////////////////////////////////////////////////////////////

bool
SCLocalization::ringCondition(double x,
                              double y,
                              double z,
                              double x_axis_origin = 0,
                              double ring_min_radius = 3,
                              double ring_max_radius = 50,
                              double ring_height = 100)
{

    // the ring is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
    if  ((!(( x >= (- x_axis_origin - ring_height) && x <= (x_axis_origin + ring_height)) && // within the Z limits
        (( pow(z,2) + pow(y,2) ) <= pow(ring_min_radius,2)))) && // within the min radius limits
        ((( x >= (- x_axis_origin - ring_height) && x <= (x_axis_origin + ring_height)) && // within the Z limits
        (( pow(z,2) + pow(y,2) ) <= pow(ring_max_radius,2))))) // within the max radius limits
  {
    return true;
  }
  else
  {
    return false;
  }

}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::Filter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  out_cloud_ptr->points.clear();

  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    // out_cloud_ptr->points.push_back(*it);


    // This is a floor condition where points above the floor are selected based on height, this has not been yet implemented in a function or a filter.
    // Can I try to filter the points that have been detected as floor points by the floor detection nodelet.
    // And is there any reason for me to do this?
    // if ( it->z >= 0.2)
    // {
    //   out_cloud_ptr->points.push_back(*it);
    // }


    g_x = it->x;
    g_y = it->y;
    g_z = it->z;
    
    // if (cylinderCondition(x, y, z))
    // {
    //   out_cloud_ptr->points.push_back(*it);
    // }

    if (cylinderCondition(g_x, g_y, g_z, 0, 4, 100) && radiusCondition(g_x, g_y, g_z, 0, 50))
    {
      out_cloud_ptr->points.push_back(*it);
    }

    // if (ringCondition(x, y, z))
    // {
    //   out_cloud_ptr->points.push_back(*it);
    // }



  }

  int totalInputPoints = in_cloud_ptr->size();
  int totalOutputPoints = out_cloud_ptr->size();
  int filteredPoints = totalInputPoints - totalOutputPoints;

  std::cout << "Total number of input points: " << std::endl;
  std::cout << totalInputPoints << std::endl;
  std::cout << "Total number of output points: " << std::endl;
  std::cout << totalOutputPoints << std::endl;
  std::cout << "Total number of filtered points: " << std::endl;
  std::cout << filteredPoints << std::endl;

}




////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::cylinderFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                double x_axis_origin = 0,
                                double radius = 3,
                                double height = 100)
{
  // Formula for the Volume of a cylinder: M_PI * (radius^2) * height
  // Formula for the Radius of a cylinder: sqrt(Volume / (M_PI * height))
  // Formula for the Height of a cylinder: Volume / (M_PI * (radius^2))
  // Formula for the Diameter of a cylinder: (sqrt(Volume / (M_PI * height)))/2

  out_cloud_ptr->points.clear();
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    // the cylinder is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
    if (!(( it->x >= (-x_axis_origin - height) && it->x <= (x_axis_origin + height)) && // within the Z limits
         (( pow(it->z,2) + pow(it->y,2) ) <= pow(radius,2)))) // within the radius limits
    {
      out_cloud_ptr->points.push_back(*it);
    }

  }

  int totalInputPoints = in_cloud_ptr->size();
  int totalOutputPoints = out_cloud_ptr->size();
  int filteredPoints = totalInputPoints - totalOutputPoints;

  std::cout << "Total number of input points: " << std::endl;
  std::cout << totalInputPoints << std::endl;
  std::cout << "Total number of output points: " << std::endl;
  std::cout << totalOutputPoints << std::endl;
  std::cout << "Total number of filtered points: " << std::endl;
  std::cout << filteredPoints << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::radiusFilter( PointCPtr &in_cloud_ptr,
                              PointCPtr &out_cloud_ptr,
                              double min_radius = 0,
                              double max_radius = 250)
{

  out_cloud_ptr->points.clear();
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    
    if ((( pow(it->x,2) + pow(it->y,2) ) >= pow(min_radius,2)) && (( pow(it->x,2) + pow(it->y,2) ) <= pow(max_radius,2)))
    {
      out_cloud_ptr->points.push_back(*it);
    }

  }

  int totalInputPoints = in_cloud_ptr->size();
  int totalOutputPoints = out_cloud_ptr->size();
  int filteredPoints = totalInputPoints - totalOutputPoints;

  std::cout << "Total number of input points: " << std::endl;
  std::cout << totalInputPoints << std::endl;
  std::cout << "Total number of output points: " << std::endl;
  std::cout << totalOutputPoints << std::endl;
  std::cout << "Total number of filtered points: " << std::endl;
  std::cout << filteredPoints << std::endl;
}


////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::ringFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                double x_axis_origin = 0,
                                double ring_min_radius = 3,
                                double ring_max_radius = 50,
                                double ring_height = 100)
{
  // This ring filter will remove points in the inner cylinder and retain points within the outter cylinder as this combines certain properties of the cylinder and radius filters
  out_cloud_ptr->points.clear();
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    // the ring is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
    if  ((!(( it->x >= (- x_axis_origin - ring_height) && it->x <= (x_axis_origin + ring_height)) && // within the Z limits
        (( pow(it->z,2) + pow(it->y,2) ) <= pow(ring_min_radius,2)))) && // within the min radius limits
        ((( it->x >= (- x_axis_origin - ring_height) && it->x <= (x_axis_origin + ring_height)) && // within the Z limits
        (( pow(it->z,2) + pow(it->y,2) ) <= pow(ring_max_radius,2))))) // within the max radius limits
    {
      out_cloud_ptr->points.push_back(*it);
    }

  }

  int totalInputPoints = in_cloud_ptr->size();
  int totalOutputPoints = out_cloud_ptr->size();
  int filteredPoints = totalInputPoints - totalOutputPoints;

  std::cout << "Total number of input points: " << std::endl;
  std::cout << totalInputPoints << std::endl;
  std::cout << "Total number of output points: " << std::endl;
  std::cout << totalOutputPoints << std::endl;
  std::cout << "Total number of filtered points: " << std::endl;
  std::cout << filteredPoints << std::endl;
  std::cout << in_cloud_ptr->header.frame_id;
}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::boxFilter(PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr,
                          double x_axis_min = -2.5, double x_axis_max = 0,
                          double y_axis_min = -20, double y_axis_max = 25,
                          double z_axis_min = -20, double z_axis_max = 25)
{
  out_cloud_ptr->points.clear();
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    if ( it->z >= z_axis_min && it->z <= z_axis_max && // within the Z limits
         it->x >= x_axis_min && it->x <= x_axis_max && it->y >= y_axis_min && it->y <= y_axis_max ) // Not within the central limits
    {
       out_cloud_ptr->points.push_back(*it);
    } 
  } 
}


////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::Add(PointCPtr &cloud1, PointCPtr &cloud2)
{
  for ( PointC::iterator it = cloud1->begin(); it != cloud1->end(); it++)
  {
    cloud2->points.push_back(*it);
  } 
  
}


////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{// declare type
  PointCPtr cloud1f(new PointC);
  PointCPtr cloud1(new PointC);
  PointCPtr cloud2(new PointC);
  PointCPtr cloud_all(new PointC);

  // These are the two synced clouds we are subscribing to, however both of these are currently same
  pcl::fromROSMsg(*cloud_msg1, *cloud1f);
  //pcl::fromROSMsg(*cloud_msg2, *cloud2);
  //////////////////////////////////////////////////
  // add both clouds
  Filter(cloud1f, cloud_all); //removed suspected unneccesary points

  // Floor Removal Tests --> Try these with and without removing floor
  // Filter(cloud1f, cloud_all); //removed suspected unneccesary points
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // radiusFilter(cloud1f, cloud_all, 0, 50); //removed suspected unneccesary points in form of radius filter
  // ringFilter(cloud1f, cloud_all, 0, 3, 50, 100); //removed suspected unneccesary points in form of ring filter

  // Cylinder Filters
  // To test best thickness of forward points to be eliminated
  // cylinderFilter(cloud1f, cloud_all, 0, 2, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 4, 100); //removed suspected unneccesary points in form of cylinder filter

  // To test range of points required to be removed
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 10); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 20); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 30); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 40); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 0, 3, 50); //removed suspected unneccesary points in form of cylinder filter

  // To test where to start the cylinder filter from
  // cylinderFilter(cloud1f, cloud_all, 2, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 5, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 10, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 15, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 20, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 30, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(cloud1f, cloud_all, 40, 3, 100); //removed suspected unneccesary points in form of cylinder filter


  // Radius Filters
  // To test inner radius of points required to be removed
  // radiusFilter(cloud1f, cloud_all, 0, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 2, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 4, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 6, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 8, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 10, 50); //removed suspected unneccesary points in form of radius filter

  // To test outer radius of points required to be retained
  // radiusFilter(cloud1f, cloud_all, 0, 40); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 0, 30); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 0, 20); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(cloud1f, cloud_all, 0, 10); //removed suspected unneccesary points in form of radius filter


  // Ring Filters // Test based on best height range from cylinder filter, range from radius filter, and inner radius of cylinder 
  // ringFilter(cloud1f, cloud_all, 0, 3, 50, 100); //removed suspected unneccesary points in form of ring filter
  

  // Add filter to remove moveable objects, (can either cluster or also check if the point is where we predicted it to be from the previous frame?)

  // Add filter to remove points that have not had an enough angle diviation from previous frame or N frames ago


  // // Previous condition -->
  // Filter(cloud1f, cloud1); //removed suspected unneccesary points
  // // Why do we need to add these clouds?
  // Add(cloud1, cloud2);// add rm floor
  // Add(cloud2, cloud_all);



  // write head and publish output
  sensor_msgs::PointCloud2 output;
  
  pcl::toROSMsg(*cloud_all, output);
  output.header = cloud_msg1->header;
  pub_.publish (output);
}

////////////////////////////////////////////////////////////////////////////////

double
SCLocalization::getVelocity(const geometry_msgs::Twist& robot_twist)
{

  return 0.0;

}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::odom_callback(const nav_msgs::OdometryConstPtr& odom_in)
{
  // Need to use this to predict where an observed point was in the previous frame
  // Can do this by estimating the relative velocity of a point based on its distance from the robot,
  // and use that to estimate its location from the previous frame
  // can use this to find the angle difference of the point in successive frames
  // can store history of velocity of the robot in a ros topic or param to check if the angle diviation
  // of a point from its last observation has been greater than a set threshold to decide if the point will be useful.
  
  // As hdl is a graph based slam algorithm, think of COMP0130 coursework of how points were eliminated there and look at coursework feedback.

  nav_msgs::Odometry robot_odom;
  robot_odom = *odom_in;
  geometry_msgs::Twist robot_twist = robot_odom.twist.twist;
  double velocity = getVelocity(robot_twist);

  //need to convert position of robot and observation to world frame to find its previous position of robot based on relative velocity
  // the position of observation does not change in the world frame 
  // (Exception is where the observation is not stationary and may need to check if the point is aligned with the map?)

  // Code to change the coordinate frame of the point cloud:

  // // Extract inout point cloud info
  // g_lidar_frame_id_ = cloud_in->header.frame_id;

  // sensor_msgs::PointCloud2 temp_cloud;
  // pcl::toROSMsg(*cloud_in, temp_cloud);
  // sensor_msgs::PointCloud cloud_lidar;
  // sensor_msgs::convertPointCloud2ToPointCloud(temp_cloud, cloud_lidar);
  // cloud_lidar.header.frame_id = g_lidar_frame_id_;
  // cloud_lidar.header.stamp = ros::Time (0);
  // sensor_msgs::PointCloud cloud_world;
  // try
  // {
  //   g_listener_.transformPointCloud ("world",
  //                                     cloud_lidar,
  //                                     cloud_world);
                                
  // }
  // catch (tf::TransformException& ex)
  // {
  //   ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  // }

  // sensor_msgs::convertPointCloudToPointCloud2(cloud_world, temp_cloud);
  // PointC cloud_world;
  // pcl::fromROSMsg(temp_cloud, cloud_world);


  // Think of ways I can access the graphical information in hdl, such as associated covariance of each observation
  // Go through CW2 and CW3 of COMP0130 on 11/06/2022


  // can actually change this to imu callback to access velocity of the robot directly as odometry twist is not published:
  // Topic of imu --> /kitti/oxts/imu


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


  std::cout<<(g_roll * (180.0/3.141592653589793238463))<<std::endl;
  std::cout<<(g_pitch * (180.0/3.141592653589793238463))<<std::endl;
  std::cout<<(g_yaw * (180.0/3.141592653589793238463))<<std::endl;

  g_rpy.push_back(to_string(g_roll));
  g_rpy.push_back(to_string(g_pitch));
  g_rpy.push_back(to_string(g_yaw));

	// Appending the Values of Roll, Pitch and Yaw to a text file.
	// std::ofstream output_file("~/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/RPY.txt");
	std::ofstream output_file("RPY.txt");
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
  SCLocalization SCLObject (nh);
  
   
  ros::spin ();
  
  return (0);

} 