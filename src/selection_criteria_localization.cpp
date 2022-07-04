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
  // odom_abs_pub = nh.advertise<nav_msgs::Odometry> ("/odom_abs", 3); // cannot do this, need to work on rotating the odometry not the pointcloud.


  // // Create a ROS subscriber for the input point cloud and floor
  // http://wiki.ros.org/message_filters?distro=melodic#Time_Synchronizer
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c1(nh_, "/points_input", 1);
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c2(nh_, "/points_input", 1);
  // // TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(c1, c1, 10);
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), c1, c2);
  
  // sync.registerCallback(boost::bind(&SCLocalization::callback, this, _1, _2));



  // Create a ROS subscriber for the input point cloud
  // sub_ = nh_.subscribe("/filtered_points_without_floor", 3, &SCLocalization::callback, this);
  sub_ = nh_.subscribe("/points_input", 3, &SCLocalization::callback, this);

  // Create a ROS subscriber for the point cloud for the floor
  // floor_sub_ = nh_.subscribe("/floor_points", 3, &SCLocalization::callback, this);
  
  // Create a ROS subscriber for computed odometry
  // odom_sub_ = nh_.subscribe("/odom", 1, &SCLocalization::odom_callback, this);
  // odom_sub_ = nh_.subscribe("/odom_transformed", 1, &SCLocalization::odom_callback, this);

  // Get name of evaluation dataset and sequence:
  ros::param::get("/dataset", g_dataset);
  ros::param::get("/sequence", g_sequence);

}


////////////////////////////////////////////////////////////////////////////////

bool
SCLocalization::cylinderCondition(double x,
                                  double y,
                                  double z,
                                  float x_axis_origin = 0,
                                  float radius = 3,
                                  float height = 100)
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
                                float min_radius = 0,
                                float max_radius = 250)
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
                              float x_axis_origin = 0,
                              float ring_min_radius = 3,
                              float ring_max_radius = 50,
                              float ring_height = 100)
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

bool
SCLocalization::floorFilter(bool filter_floor = false)
{

  // This is a floor condition where points above the floor are selected based on height, that are far away from the robot, this has not been yet implemented in a function or a filter.
  // Can I try to filter the points that have been detected as floor points by the floor detection nodelet.
  // And is there any reason for me to do this?
  // Investigate if points far away or close from the floor are more useful to filter and conclude why  if  (filter) {

  // float retained_radius = 5;
  // float retained_radius = 10;
  float retained_radius = 15;
  // float retained_radius = 20;
  // float retained_radius = 25;


  // float min_retained_radius = 2;
  // float min_retained_radius = 5;
  // float min_retained_radius = 10;
  // float max_retained_radius = 10;
  // float max_retained_radius = 15;
  // float max_retained_radius = 20;
  // float max_retained_radius = 25;


  //This height needs to be converted from the map frame to the velo frame value
  // To simplify this for now, base link z value is: 0.93 and velo link z value is 0.802724058277
  // Therefore z value of 0.005 in velo link is 0.1223 in base link
  float floor_height = 0.50;
  //BECAUSE OF THE PLANE BEING INCLINED, CANNOT GET RID OF ALL POINTS BASED ON THIS SIMPLE HEIGHT THRESHOLD SO USING A HIGHER VALUE
  //Say in the future it can be improved by integrating the usage of floor detection

  if(filter_floor == true)
  {
    // if ((g_z >= floor_height)||((( pow(g_x,2) + pow(g_y,2) ) >= pow(min_retained_radius,2)) && (( pow(g_x,2) + pow(g_y,2) ) <= pow(max_retained_radius,2))))
    if ((g_z >= floor_height)||(( pow(g_x,2) + pow(g_y,2) ) <= pow(retained_radius,2)))
    {
      return true;
    }
    else
    {
      return false;
    }

  }
  else
  {
    return true;
  }


}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::updateROSParams()
{


  ros::param::set("/filter_name", g_filter_name);

  ros::param::get("/total_input_points", g_total_input_points);
  g_total_input_points += g_in_cloud_size;
  ros::param::set("/total_input_points", g_total_input_points);
  
  ros::param::get("/total_output_points", g_total_output_points);
  g_total_output_points += g_out_cloud_size;
  ros::param::set("/total_output_points", g_total_output_points);

  ros::param::get("/total_filtered_points", g_total_filtered_points);
  g_total_filtered_points += (g_in_cloud_size - g_out_cloud_size);
  ros::param::set("/total_filtered_points", g_total_filtered_points);

  ros::param::get("/number_of_frames", g_total_number_of_frames);
  g_total_number_of_frames += 1;
  ros::param::set("/number_of_frames", g_total_number_of_frames);


  // vector<double> previous_robot_world_frame_coordinate{ g_robot_world_frame_coordinate.point.x, g_robot_world_frame_coordinate.point.y, g_robot_world_frame_coordinate.point.z };
  // previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::set("/previous_robot_world_frame_coordinate_x", g_robot_world_frame_coordinate.point.x);
  ros::param::set("/previous_robot_world_frame_coordinate_y", g_robot_world_frame_coordinate.point.y);
  ros::param::set("/previous_robot_world_frame_coordinate_z", g_robot_world_frame_coordinate.point.z);

}
////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::computeFilteredPointsData()
{

  double g_average_input_points = ((1.0 * g_total_input_points)/(1.0 * g_total_number_of_frames));
  double g_average_output_points = ((1.0 * g_total_output_points)/(1.0 * g_total_number_of_frames));
  double g_average_filtered_points = ((1.0 * g_total_filtered_points)/(1.0 * g_total_number_of_frames));

  cout << "Total number of input points: " << endl;
  cout << g_total_input_points << endl;
  cout << "Total number of output points: " << endl;
  cout << g_total_output_points << endl;
  cout << "Total number of filtered points: " << endl;
  cout << g_total_filtered_points << endl;
  cout << "Average number of input points per frame: " << endl;
  cout << g_average_input_points << endl;
  cout << "Average number of output points per frame: " << endl;
  cout << g_average_output_points << endl;
  cout << "Average number of filtered points per frame: " << endl;
  cout << g_average_filtered_points << endl;

  // defining array of filtered points data
  string filtered_points_data[6] = { "Total number of input points: " + to_string(g_total_input_points),
                            "Total number of output points: " + to_string(g_total_output_points),
                            "Total number of filtered points: " + to_string(g_total_filtered_points),
                            "Average number of input points per frame: " + to_string(g_average_input_points),
                            "Average number of output points per frame: " + to_string(g_average_output_points),
                            "Average number of filtered points per frame: " + to_string(g_average_filtered_points) };

  // get array size
  int arraySize = *(&filtered_points_data + 1) - filtered_points_data;


  g_file_name = g_dataset + "_" + g_sequence + "_" + g_filter_name;

  if (g_filter_floor)
  {
    g_file_name = g_file_name + "_ff";
  }

  g_file_name = g_file_name + ".txt";


  //exception handling
  try {
    cout << "\nSaving Filtered Points Data to file " + g_file_name;
    // Opening File
    ofstream fw("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/statistics/points/" + g_file_name, ofstream::out);
    
    // If file opened write contents
    if (fw.is_open())
    {
      //store array contents to text file
      for (int i = 0; i < arraySize; i++) {
        fw << filtered_points_data[i] << "\n";
      }
      fw.close();
    }
    else cout << "Problem with opening file";
  }
  catch (const char* msg) {
    cerr << msg << endl;
  }


}
////////////////////////////////////////////////////////////////////////////////
void
SCLocalization::transformRobotCoordinates()
{
  // Transform the point to new frame

  //Make Lidar frame id a global variable that can be adjusted from a yaml file alonside other variables later
  g_robot_lidar_frame_coordinate.header.frame_id = "velo_link";
  g_robot_lidar_frame_coordinate.header.stamp = ros::Time (0);
  g_robot_lidar_frame_coordinate.point.x = 0.0;
  g_robot_lidar_frame_coordinate.point.y = 0.0;
  g_robot_lidar_frame_coordinate.point.z = 0.0;


  
  geometry_msgs::TransformStamped transformStamped;
  ros::Duration cache_(5);
  tf2_ros::Buffer tfBuffer(cache_);
  // tf2_ros::TransformListener tfListener(tfBuffer);
  //CHECK IF THIS WORKS OR WILL HAVE TO INCLUDE BUFFER AGAIN SOMEHOW


  try
  {
    transformStamped = tfBuffer.lookupTransform("map", "velo_link",ros::Time(0));
    
    tf2::doTransform(g_robot_lidar_frame_coordinate, g_robot_world_frame_coordinate, transformStamped);
      
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }


}

////////////////////////////////////////////////////////////////////////////////
void
SCLocalization::transformPointCoordinates()
{
  // Transform the point to new frame

  g_point_lidar_frame_coordinate.header.frame_id = "velo_link";
  g_point_lidar_frame_coordinate.header.stamp = ros::Time (0);
  g_point_lidar_frame_coordinate.point.x = g_x;
  g_point_lidar_frame_coordinate.point.y = g_y;
  g_point_lidar_frame_coordinate.point.z = g_z;

  g_point_world_frame_coordinate = g_point_lidar_frame_coordinate;
  g_point_world_frame_coordinate.point.x = g_robot_world_frame_coordinate.point.x;
  g_point_world_frame_coordinate.point.y = g_robot_world_frame_coordinate.point.y;
  g_point_world_frame_coordinate.point.z = g_robot_world_frame_coordinate.point.z;

}



////////////////////////////////////////////////////////////////////////////////
double
SCLocalization::computeAngleDeviation()
{

  double angle = 0.0;


  // Transform the points to new frame
  transformPointCoordinates();


  g_robot_world_frame_coordinate;
  g_previous_robot_world_frame_coordinate;
  g_point_world_frame_coordinate;


  g_vdt[0] = g_robot_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x;
  g_vdt[1] = g_robot_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y;
  g_vdt[2] = g_robot_world_frame_coordinate.point.z - g_previous_robot_world_frame_coordinate.point.z;


  g_d1[0] = g_point_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x;
  g_d1[1] = g_point_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y;
  g_d1[2] = g_point_world_frame_coordinate.point.z - g_previous_robot_world_frame_coordinate.point.z;

  g_d2[0] = g_point_world_frame_coordinate.point.x - g_robot_world_frame_coordinate.point.x;
  g_d2[1] = g_point_world_frame_coordinate.point.y - g_robot_world_frame_coordinate.point.y;
  g_d2[2] = g_point_world_frame_coordinate.point.z - g_robot_world_frame_coordinate.point.z;


  g_mod_vdt = sqrt(((g_vdt[0])*(g_vdt[0])) + ((g_vdt[1])*(g_vdt[1])) + ((g_vdt[2])*(g_vdt[2])));
  g_mod_d1 = sqrt(((g_d1[0])*(g_d1[0])) + ((g_d1[1])*(g_d1[1])) + ((g_d1[2])*(g_d1[2])));
  g_mod_d2 = sqrt(((g_d2[0])*(g_d2[0])) + ((g_d2[1])*(g_d2[1])) + ((g_d2[2])*(g_d2[2])));

  // double mod_vdt_sqr = pow(mod_vdt,2);
  g_mod_d1_sqr = pow(g_mod_d1,2);
  g_mod_d2_sqr = pow(g_mod_d2,2);

  angle = acos((g_mod_d1_sqr + g_mod_d2_sqr - g_mod_vdt)/(2*g_mod_d1*g_mod_d2));


  // cout << "The angle deviation of the current point is: " << endl;
  // cout << angle << endl;

  return angle;

}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::Filter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  out_cloud_ptr->points.clear();


  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);

  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;


    double angle = computeAngleDeviation();


    // out_cloud_ptr->points.push_back(*it);


    if (floorFilter(g_filter_floor))
    {
      out_cloud_ptr->points.push_back(*it);

      // if (cylinderCondition(x, y, z))
      // {
      //   out_cloud_ptr->points.push_back(*it);
      // }

      // if (cylinderCondition(g_x, g_y, g_z, 0, 4, 100) && radiusCondition(g_x, g_y, g_z, 0, 50) && (g_z >= 0.05))
      // {
      //   out_cloud_ptr->points.push_back(*it);
      // }

      // if (ringCondition(x, y, z))
      // {
      //   out_cloud_ptr->points.push_back(*it);
      // }

    }




  }

  g_filter_name = "vanilla";
  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  updateROSParams();
  computeFilteredPointsData();


}




////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::cylinderFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                float x_axis_origin = 0,
                                float radius = 3,
                                float height = 100)
{
  // Formula for the Volume of a cylinder: M_PI * (radius^2) * height
  // Formula for the Radius of a cylinder: sqrt(Volume / (M_PI * height))
  // Formula for the Height of a cylinder: Volume / (M_PI * (radius^2))
  // Formula for the Diameter of a cylinder: (sqrt(Volume / (M_PI * height)))/2

  out_cloud_ptr->points.clear();


  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    g_x = it->x;
    g_y = it->y;
    g_z = it->z;

    double angle = computeAngleDeviation();
    

    if (floorFilter(g_filter_floor))
    {
      // the cylinder is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
      if (!(((( (-height <= g_x) && (g_x <= -x_axis_origin) ) || ((x_axis_origin <= g_x) && (g_x <=  height)))) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(radius,2)))) // within the radius limits
      {
        out_cloud_ptr->points.push_back(*it);
      }
    }

  }
  


  ros::param::get("/filter_name", g_filter_name);
  
  if (g_filter_name.empty())
  {
    g_filter_name = string("cyl") + string("_") + to_string(x_axis_origin) + string("_") + to_string(radius) + string("_") + to_string(height);
  }
  else if (g_filter_name.find("cyl") != string::npos)
  {
    g_filter_name = g_filter_name;
  }
  else
  {
    g_filter_name = g_filter_name + string("_") + string("cyl") + string("_") + to_string(x_axis_origin) + string("_") + to_string(radius) + string("_") + to_string(height);
  }
  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  updateROSParams();
  computeFilteredPointsData();


}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::radiusFilter( PointCPtr &in_cloud_ptr,
                              PointCPtr &out_cloud_ptr,
                              float min_radius = 0,
                              float max_radius = 250)
{

  out_cloud_ptr->points.clear();


  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;


    double angle = computeAngleDeviation();
    

    if (floorFilter(g_filter_floor))
    {
      if ((( pow(g_x,2) + pow(g_y,2) ) >= pow(min_radius,2)) && (( pow(g_x,2) + pow(g_y,2) ) <= pow(max_radius,2)))
      {
        out_cloud_ptr->points.push_back(*it);
      }
    }

  }

  

  ros::param::get("/filter_name", g_filter_name);
  
  if (g_filter_name.empty())
  {
    g_filter_name = string("rad") + string("_") + to_string(min_radius) + string("_") + to_string(max_radius);
  }
  else if (g_filter_name.find("rad") != string::npos)
  {
    g_filter_name = g_filter_name;
  }  
  else
  {
    g_filter_name = g_filter_name + string("_") + string("rad") + string("_") + to_string(min_radius) + string("_") + to_string(max_radius);
  }

  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  updateROSParams();
  computeFilteredPointsData();


}


////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::ringFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                float x_axis_origin = 15,
                                float ring_min_radius = 2,
                                float ring_max_radius = 40,
                                float ring_height = 50)
{
  // This ring filter will remove points in the inner cylinder and retain points within the outter cylinder as this combines certain properties of the cylinder and radius filters
  out_cloud_ptr->points.clear();


  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;



    double angle = computeAngleDeviation();


    if (floorFilter(g_filter_floor))
    {
      // the ring is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
      if ((!(( g_x >= (-x_axis_origin - ring_height) && g_x <= (x_axis_origin + ring_height)) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(ring_min_radius,2)))) && // within the min radius limits
          ((( g_x >= (-ring_height) && g_x <= (ring_height)) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(ring_max_radius,2)))) // within the max radius limits
        )
      {
        out_cloud_ptr->points.push_back(*it);
      }
    }

  }

  

  ros::param::get("/filter_name", g_filter_name);
  
  if (g_filter_name.empty())
  {
    g_filter_name = string("ring") + string("_") + to_string(x_axis_origin) + string("_") + to_string(ring_min_radius) 
                    + string("_") + to_string(ring_max_radius) + string("_") + to_string(ring_height);    
  }
  else if (g_filter_name.find("ring") != string::npos)
  {
    g_filter_name = g_filter_name;
  }  
  else
  {
    g_filter_name = g_filter_name + string("_") + string("ring") + string("_") + to_string(x_axis_origin) + string("_") + to_string(ring_min_radius) 
                    + string("_") + to_string(ring_max_radius) + string("_") + to_string(ring_height);    
  }
  
  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  updateROSParams();
  computeFilteredPointsData();


}

////////////////////////////////////////////////////////////////////////////////

void
SCLocalization::boxFilter(PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr,
                          float x_axis_min = -30, float x_axis_max = 30,
                          float y_axis_min = -3, float y_axis_max = 3,
                          float z_axis_min = -100, float z_axis_max = 100)
{
  out_cloud_ptr->points.clear();
    
  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);

  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;

    double angle = computeAngleDeviation();

    if (floorFilter(g_filter_floor))
    {

      if (!( ((g_z >= z_axis_min) && (g_z <= z_axis_max)) &&
            ((g_x >= x_axis_min) && (g_x <= x_axis_max)) &&
            ((g_y >= y_axis_min) && (g_y <= y_axis_max)))) // Not within the box limits
      {
        out_cloud_ptr->points.push_back(*it);
      }
    }

  }

  

  ros::param::get("/filter_name", g_filter_name);
  
  if (g_filter_name.empty())
  {

    g_filter_name = string("box") + string("_")
                        + to_string(x_axis_min) + string("_") + to_string(x_axis_max)
                        + to_string(y_axis_min) + string("_") + to_string(y_axis_max)
                        + to_string(z_axis_min) + string("_") + to_string(z_axis_max);
  
  }
  else if (g_filter_name.find("box") != string::npos)
  {
    g_filter_name = g_filter_name;
  }  
  else
  {
    g_filter_name = g_filter_name + string("_") + string("box") + string("_")
                        + to_string(x_axis_min) + string("_") + to_string(x_axis_max)
                        + to_string(y_axis_min) + string("_") + to_string(y_axis_max)
                        + to_string(z_axis_min) + string("_") + to_string(z_axis_max);
  }

  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  updateROSParams();
  computeFilteredPointsData();


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
SCLocalization::callback(const sensor_msgs::PointCloud2ConstPtr& filtered_cloud_msg)
// SCLocalization::callback(const sensor_msgs::PointCloud2ConstPtr& filtered_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& floor_cloud_msg)
{// declare type
  PointCPtr filtered_cloud(new PointC);
  // PointCPtr floor_cloud(new PointC);
  PointCPtr cloud1(new PointC);
  PointCPtr cloud2(new PointC);
  PointCPtr cloud3(new PointC);
  
  PointCPtr cloud_out(new PointC);

  // These are the two synced clouds we are subscribing to, however both of these are currently same
  pcl::fromROSMsg(*filtered_cloud_msg, *filtered_cloud);
  // pcl::fromROSMsg(*floor_cloud_msg, *floor_cloud);
  //////////////////////////////////////////////////

  // Bool to determine weather to filter the floor.
  // Need to check how many more points apart from the floor are filtered by my filters
  g_filter_floor = true;

  // Filter(filtered_cloud, cloud_out); //removed suspected unneccesary points

  // Explain the naming convension of the test files properly in the thesis.

  // Floor Removal Tests --> Try these with and without removing floor
  // Filter(filtered_cloud, cloud_out); //removed suspected unneccesary points
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // radiusFilter(filtered_cloud, cloud_out, 0, 50); //removed suspected unneccesary points in form of radius filter
  // ringFilter(filtered_cloud, cloud_out, 0, 3, 50, 100); //removed suspected unneccesary points in form of ring filter

  // Cylinder Filters
  // To test best thickness of forward points to be eliminated
  // cylinderFilter(filtered_cloud, cloud_out, 0, 2, 100); //removed suspected unneccesary points in form of cylinder filter
  cylinderFilter(filtered_cloud, cloud_out, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 4, 100); //removed suspected unneccesary points in form of cylinder filter --> works as the best cyliner filter with almost 45% of filtered cloud
  // cylinderFilter(filtered_cloud, cloud_out, 0, 5, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 6, 100); //removed suspected unneccesary points in form of cylinder filter --> note in report that 6 fails

  // To test range of points required to be removed
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 10); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 20); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 30); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 40); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 0, 3, 50); //removed suspected unneccesary points in form of cylinder filter

  // To test where to start the cylinder filter from
  // cylinderFilter(filtered_cloud, cloud_out, 2, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 5, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 10, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 15, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 20, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 30, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 40, 3, 100); //removed suspected unneccesary points in form of cylinder filter

  // To test best range of forward points to be eliminated
  // cylinderFilter(filtered_cloud, cloud_out, 80, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 60, 3, 80); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 40, 3, 60); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, 20, 3, 40); //removed suspected unneccesary points in form of cylinder filter
  // The height of the filter here should be based on results here and the test from the range evaluations, so i can add them here
  // 0 to 20 is already tested previously, use it again here


  // Radius Filters
  // To test inner radius of points required to be removed
  // radiusFilter(filtered_cloud, cloud_out, 0, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 2, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 4, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 6, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 8, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 10, 50); //removed suspected unneccesary points in form of radius filter

  // To test outer radius of points required to be retained
  // USe the inner radius herre based on the best inner radius identified and add some more tests to this
  // radiusFilter(filtered_cloud, cloud_out, 0, 40); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 0, 30); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, 0, 20); //removed suspected unneccesary points in form of radius filter

  // Trying min and max radius parameters that worked quite well individually and comparing performance against all other radius filters.
  // radiusFilter(filtered_cloud, cloud_out, 8, 30); //removed suspected unneccesary points in form of radius filter


  // Ring Filters // Test based on best height range from cylinder filter, range from radius filter, and inner radius of cylinder 
  // ringFilter(filtered_cloud, cloud_out, 15, 2, 40, 50); //removed suspected unneccesary points in form of ring filter


  // Box Filters // Test based on best height range from cylinder filter, range from radius filter, and inner radius of cylinder 
  // boxFilter(filtered_cloud, cloud_out, -30, 30, -3, 3, -100, 100); //removed suspected unneccesary points in form of ring filter
  

  // Add filter to remove moveable objects, (can either cluster or also check if the point is where we predicted it to be from the previous frame?)

  // Add filter to remove points that have not had an enough angle diviation from previous frame or N frames ago


  // // Previous condition -->
  // Filter(filtered_cloud, cloud1); //removed suspected unneccesary points
  // // Why do we need to add these clouds?
  // Add(cloud1, floor_cloud);// add rm floor
  // Add(floor_cloud, cloud_out);



  // write head and publish output
  sensor_msgs::PointCloud2 output;
  
  pcl::toROSMsg(*cloud_out, output);
  output.header = filtered_cloud_msg->header;
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
  // geometry_msgs::Twist robot_twist = robot_odom.twist.twist;
  // double velocity = getVelocity(robot_twist);


  // // Find lidar and robot coordinate using odometry message, as using transform points increases the computational complexity alot for some reason and slam fails.
  // // But if i can fix this problem, it would be much more accurate
  // The problem was because of trying to call a rosparm from the server whilte itterating through each point from the cloud, therefore it could not keep up!

  // //Make Lidar frame id a global variable that can be adjusted from a yaml file alonside other variables later
  // g_robot_lidar_frame_coordinate.header.frame_id = "velo_link";
  // g_robot_lidar_frame_coordinate.header.stamp = ros::Time (0);
  // g_robot_lidar_frame_coordinate.point.x = 0.0;
  // g_robot_lidar_frame_coordinate.point.y = 0.0;
  // g_robot_lidar_frame_coordinate.point.z = 0.0;  

  // g_robot_world_frame_coordinate = g_robot_lidar_frame_coordinate;
  // g_robot_world_frame_coordinate.point.x = robot_odom.pose.pose.position.x;
  // g_robot_world_frame_coordinate.point.y = robot_odom.pose.pose.position.y;
  // g_robot_world_frame_coordinate.point.z = robot_odom.pose.pose.position.z;

  // g_point_lidar_frame_coordinate.header.frame_id = "velo_link";
  // g_point_lidar_frame_coordinate.header.stamp = ros::Time (0);
  // g_point_lidar_frame_coordinate.point.x = g_x;
  // g_point_lidar_frame_coordinate.point.y = g_y;
  // g_point_lidar_frame_coordinate.point.z = g_z;

  // g_point_world_frame_coordinate = g_point_lidar_frame_coordinate;
  // g_point_world_frame_coordinate.point.x = g_robot_world_frame_coordinate.point.x;
  // g_point_world_frame_coordinate.point.y = g_robot_world_frame_coordinate.point.y;
  // g_point_world_frame_coordinate.point.z = g_robot_world_frame_coordinate.point.z;
      


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