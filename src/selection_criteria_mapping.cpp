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

#include <selection_criteria/selection_criteria_mapping.h>


// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;



////////////////////////////////////////////////////////////////////////////////
SCMapping::SCMapping (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  debug_ (false)
{
  nh_ = nh;

  // Define the publishers
  pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/selected_points", 3, true);
  pub_vis_selected_points_ = nh_.advertise<sensor_msgs::PointCloud2> ("/vis_selected_points", 3, true);
  // odom_abs_pub = nh.advertise<nav_msgs::Odometry> ("/odom_abs", 3); // cannot do this, need to work on rotating the odometry not the pointcloud.
  // Publishing to visualize the statistics of angle deviation of points over time:
  pub_alpha_mean_ = nh_.advertise<std_msgs::Float32> ("/points_alpha_mean_", 3, true);
  pub_alpha_std_ = nh_.advertise<std_msgs::Float32> ("/points_alpha_std_", 3, true);
  pub_alpha_min_ = nh_.advertise<std_msgs::Float32> ("/points_alpha_min_", 3, true);
  pub_alpha_max_ = nh_.advertise<std_msgs::Float32> ("/points_alpha_max_", 3, true);





  // Create a ROS subscriber for the input point cloud
  // sub_ = nh_.subscribe("/filtered_points_without_floor", 3, &SCMapping::callback, this);
  sub_ = nh_.subscribe("/points_input", 3, &SCMapping::callback, this);

  // Create a ROS subscriber for the point cloud for the floor
  // floor_sub_ = nh_.subscribe("/floor_points", 3, &SCMapping::callback, this);
  
  // Create a ROS subscriber for computed odometry
  odom_sub_ = nh_.subscribe("/odom", 1, &SCMapping::odom_callback, this);
  // odom_sub_ = nh_.subscribe("/odom_transformed", 1, &SCMapping::odom_callback, this);


  // Create a ROS subscriber for recorded IMU Data
  // May need to transform the coordinate frame for this as well, but may get over this if magnitude for velocity is needed
  imu_sub_ = nh_.subscribe("/kitti/oxts/imu", 1, &SCMapping::imu_callback, this);

  // Get name of evaluation dataset and sequence:
  ros::param::get("/dataset", g_dataset);
  ros::param::get("/sequence", g_sequence);

}


////////////////////////////////////////////////////////////////////////////////

bool
SCMapping::cylinderCondition(double x,
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
  if (!(((( (-height <= x) && (x <= -x_axis_origin) ) || ((x_axis_origin <= x) && (x <=  height)))) && // within the X limits
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
SCMapping::radiusCondition(double x,
                           double y,
                           double z,
                           float min_radius = 0,
                           float max_radius = 250)
{

  if ((( pow(x,2) + pow(y,2) ) >= pow(min_radius,2)) && (( pow(x,2) + pow(y,2) ) <= pow(max_radius,2))) // within the radius limits
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
SCMapping::ringCondition(double x,
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
SCMapping::floorFilter(bool filter_floor = false)
{

  // This is a floor condition where points above the floor are selected based on height, that are far away from the robot, this has not been yet implemented in a function or a filter.


  //This height needs to be converted from the map frame to the velo frame value
  // To simplify this for now, base link z value is: 0.93 and velo link z value is 0.802724058277
  // Therefore z value of 0.005 in velo link is 0.1223 in base link

  if(filter_floor == true)
  {
    // if ((g_z >= floor_height)||((( pow(g_x,2) + pow(g_y,2) ) >= pow(g_min_retained_floor_radius,2)) && (( pow(g_x,2) + pow(g_y,2) ) <= pow(g_max_retained_floor_radius,2))))
    if ((g_z >= g_floor_height)||((( pow(g_x,2) + pow(g_y,2) ) <= pow(g_max_retained_floor_radius,2)) && (( pow(g_x,2) + pow(g_y,2) ) >= pow(g_min_retained_floor_radius,2))))
    {
      return true;
    }
    else if((g_double_floor_ring) && ((g_z >= g_floor_height)||((( pow(g_x,2) + pow(g_y,2) ) <= pow(g_max_retained_floor_radius + g_gap_to_next_floor_ring,2))
                                      && (( pow(g_x,2) + pow(g_y,2) ) >= pow(g_min_retained_floor_radius + g_gap_to_next_floor_ring,2))))) {
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
SCMapping::updateROSParams()
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


  ros::param::get("/current_time", g_previous_time);
  ros::param::set("/previous_time", g_previous_time);

  ros::param::set("/robot_previous_linear_velocity_x", g_robot_linear_velocity_x);
  ros::param::set("/robot_previous_linear_velocity_y", g_robot_linear_velocity_y);
  // ros::param::set("/robot_previous_linear_velocity_z", g_robot_linear_velocity_z);
  // ros::param::set("/robot_previous_linear_velocity_abs", g_robot_linear_velocity_abs);


  ros::param::set("/robot_previous_angular_velocity", g_robot_angular_velocity);


  ros::param::set("/robot_previous_angle", g_robot_angle);


  ros::param::set("/previous_alpha_min", alpha_min.data);
  ros::param::set("/previous_alpha_max", alpha_max.data);
  ros::param::set("/previous_alpha_mean", alpha_mean.data);
  ros::param::set("/previous_alpha_std", alpha_std.data);

  int previous_matched_alpha = g_matched_alpha;
  ros::param::set("/previous_matched_alpha", previous_matched_alpha);

  // cout << previous_matched_alpha << endl;




}
////////////////////////////////////////////////////////////////////////////////

void
SCMapping::computeFilteredPointsData()
{

  double g_average_input_points = ((1.0 * g_total_input_points)/(1.0 * g_total_number_of_frames));
  double g_average_output_points = ((1.0 * g_total_output_points)/(1.0 * g_total_number_of_frames));
  double g_average_filtered_points = ((1.0 * g_total_filtered_points)/(1.0 * g_total_number_of_frames));



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


  //Saving statistics regarding filtered points
  try {
    // cout << "\nSaving Filtered Points Data to file " + g_file_name;
    // Opening File
    ofstream fw("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/graph_slam/statistics/points/" + g_file_name, ofstream::out);
    
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

  //Saving information regarding rostime and velocity
  try {
    std::ofstream out;
    out.open("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/graph_slam/statistics/time_and_speed/" + g_file_name, std::ios::app);
    // ofstream fw("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/graph_slam/statistics/time_and_speed/" + g_file_name, std::ios_base::app);
    
    // If file opened write contents
    if (out.is_open())
    {
      //store array contents to text file
      out << to_string(g_current_time) + "," + to_string(g_robot_linear_velocity_abs) + "," + to_string(g_robot_angular_velocity) << "\n";

      out.close();
    }
    else cout << "Problem with opening file";
  }
  catch (const char* msg) {
    cerr << msg << endl;
  }


}


////////////////////////////////////////////////////////////////////////////////

void
SCMapping::saveAlphaHistogram(unsigned int histogramAlpha[900]) {

  // get array size
  int arraySize = 900;


  g_file_name = g_dataset + "_" + g_sequence + "_" + g_filter_name;

  if (g_filter_floor)
  {
    g_file_name = g_file_name + "_ff";
  }

  g_file_name = g_file_name + "_AlphaHistogram.txt";
  
  int i = 0;
  try {
    fstream inputFile;
    inputFile.open("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/graph_slam/" + g_file_name,ios::in); //open a file to perform read operation using file object
    if (inputFile.is_open()){   //checking whether the file is open
      string tp;
      while(getline(inputFile, tp)){ //read data from file object and put it into string.
          histogramAlpha[i] += stoi(tp);
          i++;
      }
      inputFile.close(); //close the file object.
    }
  }
  catch (const char* msg) {
    cerr << msg << endl;
  }


  //Saving statistics regarding filtered points
  try {
    // cout << "\nSaving Filtered Points Data to file " + g_file_name;
    // Opening File
    ofstream fw("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + g_dataset + "/" + g_sequence + "/results/graph_slam/" + g_file_name, ofstream::out);
    
    // If file opened write contents
    if (fw.is_open())
    {
      //store array contents to text file
      for (int j = 0; j < arraySize; j++) {
        fw << histogramAlpha[j] << "\n";
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
SCMapping::transformRobotCoordinates()
{
  // Transform the point to new frame

  //Make Lidar frame id a global variable that can be adjusted from a yaml file alonside other variables later
  g_robot_lidar_frame_coordinate.header.frame_id = "velo_link";
  g_robot_lidar_frame_coordinate.header.stamp = ros::Time (0);
  g_robot_lidar_frame_coordinate.point.x = 0.0;
  g_robot_lidar_frame_coordinate.point.y = 0.0;
  g_robot_lidar_frame_coordinate.point.z = 0.0;


  try
  {
    g_listener_.transformPoint ("map", 
                                g_robot_lidar_frame_coordinate,
                                g_robot_world_frame_coordinate);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }


  g_base_link_world_frame_coordinate.header.frame_id = "base_link";
  g_base_link_world_frame_coordinate.header.stamp = ros::Time (0);
  g_base_link_world_frame_coordinate.point.x = 0.0;
  g_base_link_world_frame_coordinate.point.y = 0.0;
  g_base_link_world_frame_coordinate.point.z = 0.0;


  try
  {
    g_listener_.transformPoint ("velo_link", 
                                g_base_link_world_frame_coordinate,
                                g_base_link_lidar_frame_coordinate);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }



  // g_floor_height = g_base_link_lidar_frame_coordinate.point.z + 0.3;

  g_floor_height = g_base_link_lidar_frame_coordinate.point.z + 2.13;

  
  

}

////////////////////////////////////////////////////////////////////////////////
void
SCMapping::computeTrajectoryInformation()
{
  //Compute and save additional robot trajectory information to odometry

  // Save the data regarding speed and time in a text file here.

  // V = U + AT
  // speed = distance / time

  // To get the current ros time
  g_current_time = ros::Time::now().toSec();
  ros::param::set("/current_time", g_current_time);
  // To get the previous ros time
  ros::param::get("/previous_time", g_previous_time);
  // To get the elapsed ros time
  double time_elapsed = g_current_time - g_previous_time;



  ros::param::get("/robot_previous_angular_velocity", g_robot_previous_angular_velocity);



  ros::param::get("/robot_previous_angle", g_robot_previous_angle);
  if(g_robot_angle == 0.0) {
    g_robot_angle = g_robot_previous_angle;
  }

  g_robot_angular_velocity = (abs(g_robot_angle) - abs(g_robot_previous_angle)) / time_elapsed;
  g_robot_angular_velocity = ((abs(g_robot_angular_velocity) + abs(g_robot_previous_angular_velocity)) / 2); //To get a smoother velocity chart



  // To get the robot's linear acceleration data
  // Not concidering z axis as it has acceleration due to gravity, and gives irrelevant readings.
  ros::param::get("/robot_linear_acceleration_x", g_robot_linear_acceleration_x);
  ros::param::get("/robot_linear_acceleration_y", g_robot_linear_acceleration_y);

  // To get the robot's previous linear velocity data
  ros::param::get("/robot_previous_linear_velocity_x", g_robot_previous_linear_velocity_x);
  ros::param::get("/robot_previous_linear_velocity_y", g_robot_previous_linear_velocity_y);



  // The following formula for velocity uses odometry data and no acceleration information from IMU, to check if the imu is giving reliable data:
  g_robot_linear_velocity_x = (g_robot_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x) / time_elapsed;
  g_robot_linear_velocity_x = ((g_robot_linear_velocity_x + g_robot_previous_linear_velocity_x) / 2); //To get a smoother velocity chart
  g_robot_linear_velocity_y = (g_robot_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y) / time_elapsed;
  g_robot_linear_velocity_y = ((g_robot_linear_velocity_y + g_robot_previous_linear_velocity_y) / 2); //To get a smoother velocity chart

  g_robot_linear_velocity_abs = sqrt(pow(g_robot_linear_velocity_x,2) + pow(g_robot_linear_velocity_y,2));




}

////////////////////////////////////////////////////////////////////////////////
void
SCMapping::transformPointCoordinates()
{
  // Transform the point to new frame

  g_point_lidar_frame_coordinate.header.frame_id = "velo_link";
  g_point_lidar_frame_coordinate.header.stamp = ros::Time (0);
  g_point_lidar_frame_coordinate.point.x = g_x;
  g_point_lidar_frame_coordinate.point.y = g_y;
  g_point_lidar_frame_coordinate.point.z = g_z;

  g_point_world_frame_coordinate = g_point_lidar_frame_coordinate;
  g_point_world_frame_coordinate.point.x += g_robot_world_frame_coordinate.point.x;
  g_point_world_frame_coordinate.point.y += g_robot_world_frame_coordinate.point.y;
  g_point_world_frame_coordinate.point.z += g_robot_world_frame_coordinate.point.z;

}



////////////////////////////////////////////////////////////////////////////////
double
SCMapping::computeAlpha()
{

  double alpha = 0.0;



  g_vdt[0] = g_robot_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x;
  g_vdt[1] = g_robot_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y;
  g_vdt[2] = g_robot_world_frame_coordinate.point.z - g_previous_robot_world_frame_coordinate.point.z;


  g_d1[0] = g_point_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x;
  g_d1[1] = g_point_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y;
  g_d1[2] = g_point_world_frame_coordinate.point.z - g_previous_robot_world_frame_coordinate.point.z;

  g_d2[0] = g_point_world_frame_coordinate.point.x - g_robot_world_frame_coordinate.point.x;
  g_d2[1] = g_point_world_frame_coordinate.point.y - g_robot_world_frame_coordinate.point.y;
  g_d2[2] = g_point_world_frame_coordinate.point.z - g_robot_world_frame_coordinate.point.z;


  g_mod_vdt = sqrt(((g_vdt[0])*(g_vdt[0])) + ((g_vdt[1])*(g_vdt[1])));
  g_mod_d1 = sqrt(((g_d1[0])*(g_d1[0])) + ((g_d1[1])*(g_d1[1])));
  g_mod_d2 = sqrt(((g_d2[0])*(g_d2[0])) + ((g_d2[1])*(g_d2[1])));

  g_mod_d1_sqr = pow(g_mod_d1,2);
  g_mod_d2_sqr = pow(g_mod_d2,2);
  g_mod_vdt_sqr = pow(g_mod_vdt,2);

  // Angle between observation of the point in degrees:
  double ratio = (g_mod_d1_sqr + g_mod_d2_sqr - g_mod_vdt_sqr)/(2*g_mod_d1*g_mod_d2);
  alpha = (acos(ratio)) * 180 / M_PI;


  return alpha;

}

////////////////////////////////////////////////////////////////////////////////
void
SCMapping::computeAlphaStatistics()
{
  // Finding the sum, mean, square of sums and std of angle deviation
  // double sum = std::accumulate(alpha_vec.begin(), alpha_vec.end(), 0.0);
  // double mean = sum / alpha_vec.size();
  // double sq_sum = std::inner_product(alpha_vec.begin(), alpha_vec.end(), alpha_vec.begin(), 0.0);
  // double E = 0.0;
  // // Quick Question - Can vector::size() return 0?
  // double inverse = 1.0 / static_cast<double>(alpha_vec.size());
  // for(unsigned int i=0;i<alpha_vec.size();i++)
  // {
  //     E += pow(static_cast<double>(alpha_vec[i]) - mean, 2);
  // }
  // double stdev =  sqrt(inverse * E);

  // double sum = std::accumulate(alpha_vec.begin(), alpha_vec.end(), 0.0);
  // double mean = sum / alpha_vec.size();
  // std::vector<double> diff(alpha_vec.size());
  // std::transform(alpha_vec.begin(), alpha_vec.end(), diff.begin(),
  //               std::bind2nd(std::minus<double>(), mean));
  // double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  // double std = std::sqrt(sq_sum / alpha_vec.size());

  // Finding the sum of the vector of stored angle deviations
  double sum = 0.0;
  for(int i=0;i<alpha_vec.size();i++)
          sum+=alpha_vec[i];

  // cout << sum << endl;

  // Finding the mean of the vector of stored angle deviations
  double mean = sum/alpha_vec.size();

  // Finding the standard deviation of the vector of stored angle deviations
  double stdev = std::sqrt(std::inner_product(alpha_vec.begin(), alpha_vec.end(), alpha_vec.begin(), 0.0)
                 / alpha_vec.size() - mean * mean);



  auto minmax = std::minmax_element(alpha_vec.begin(), alpha_vec.end());
  // Smoothning the computed statistics based on previous data
  alpha_mean.data = ((mean + previous_alpha_mean) / 2);
  alpha_std.data = ((stdev + previous_alpha_std) / 2);
  alpha_min.data = ((*minmax.first + previous_alpha_min) / 2);
  alpha_max.data = ((*minmax.second + previous_alpha_max) / 2);

  // Making sure the statistics of the angle are smoothened, even in instances of unexpected erroes
  if(isnan(sum))
  {
    alpha_mean.data = previous_alpha_mean;
    alpha_std.data = previous_alpha_std;
    alpha_min.data = previous_alpha_min;
    alpha_max.data = previous_alpha_max;
  }

  // Publishing the smoothened angles
  pub_alpha_mean_.publish (alpha_mean);
  pub_alpha_std_.publish (alpha_std);
  pub_alpha_min_.publish (alpha_min);
  pub_alpha_max_.publish (alpha_max);

  // cout << alpha_max << endl;

  

}

////////////////////////////////////////////////////////////////////////////////
bool
SCMapping::computePointObservability()
{
  bool observable = false;
  
  // Checking if the point was observable in the previous frame:
  double eu_distance = sqrt(pow(g_point_world_frame_coordinate.point.x - g_previous_robot_world_frame_coordinate.point.x, 2)
                            + pow(g_point_world_frame_coordinate.point.y - g_previous_robot_world_frame_coordinate.point.y, 2));

  // Measurement Range of LiDAR: Up to 120 m
  if (eu_distance < 120) {
    observable = true;
  }

  return observable;

}



////////////////////////////////////////////////////////////////////////////////
double
SCMapping::computeObservationAngle()
{

  double observation_angle = 0.0;


  // cout << "g_robot_world_frame_coordinate: " << endl;
  // cout << g_robot_world_frame_coordinate << endl;

  // cout << "g_previous_robot_world_frame_coordinate: " << endl;
  // cout << g_previous_robot_world_frame_coordinate << endl;

  // cout << "g_point_world_frame_coordinate: " << endl;
  // cout << g_point_world_frame_coordinate << endl;


  g_dp[0] = g_point_world_frame_coordinate.point.x - g_point_world_frame_coordinate.point.x;
  g_dp[1] = g_point_world_frame_coordinate.point.y - g_point_world_frame_coordinate.point.y;
  g_dp[2] = g_point_world_frame_coordinate.point.z - g_robot_world_frame_coordinate.point.z;

  g_do[0] = g_point_world_frame_coordinate.point.x - g_robot_world_frame_coordinate.point.x;
  g_do[1] = g_point_world_frame_coordinate.point.y - g_robot_world_frame_coordinate.point.y;
  g_do[2] = g_robot_world_frame_coordinate.point.z - g_robot_world_frame_coordinate.point.z;

  g_d2[0] = g_point_world_frame_coordinate.point.x - g_robot_world_frame_coordinate.point.x;
  g_d2[1] = g_point_world_frame_coordinate.point.y - g_robot_world_frame_coordinate.point.y;
  g_d2[2] = g_point_world_frame_coordinate.point.z - g_robot_world_frame_coordinate.point.z;


  g_mod_do = sqrt(((g_do[0])*(g_do[0])) + ((g_do[1])*(g_do[1])) + ((g_do[2])*(g_do[2])));
  // cout << "g_mod_do: " << endl;
  // cout << g_mod_do << endl;

  g_mod_dp = sqrt(((g_dp[0])*(g_dp[0])) + ((g_dp[1])*(g_dp[1])) + ((g_dp[2])*(g_dp[2])));
  // cout << "g_mod_dp: " << endl;
  // cout << g_mod_dp << endl;

  g_mod_d2 = sqrt(((g_d2[0])*(g_d2[0])) + ((g_d2[1])*(g_d2[1])) + ((g_d2[2])*(g_d2[2])));
  // cout << "g_mod_d2: " << endl;
  // cout << g_mod_d2 << endl;

  // Angle between observation of the point and lidar plane in degrees:
  // Note: If angle is nan, means that poisition of point was not estimated by LiDAR.
  observation_angle = acos((g_mod_dp)/(g_mod_d2)) * 180 / M_PI;


  // cout << "The observation angle of the current point is: " << endl;
  // cout << observation_angle << endl;

  return observation_angle;

}

////////////////////////////////////////////////////////////////////////////////

void
SCMapping::Filter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, PointCPtr &vis_cloud_ptr)
{

  // Need to show fianl oerformance agianst vanilla visually, without saving metric data to show true performance and say how saving metrics have a small affect.

  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();


  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);


  // Compute and save additional robot trajectory information to odometry
  computeTrajectoryInformation();

  // g_previous_robot_world_frame_coordinate

  // cout << "g_previous_robot_world_frame_coordinate: " << endl;
  // cout << g_previous_robot_world_frame_coordinate << endl;



	// /* Angle Deviation histogram */
	// unsigned int histogramAlpha[900] = { 0 };
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;

    // out_cloud_ptr->points.push_back(*it);

    // Transform the points to new frame
    transformPointCoordinates();


    // // Computing Angle Deviation of point with respect to previous frame:
    // double alpha_idx;

    // if((not isnan(computeAlpha()))) {
    //   alpha_idx = computeAlpha()/0.1;
    // }
    // else {
    //   alpha_idx = 0;
    // }

    // // cout << alpha_idx << endl;

    // unsigned int vAlpha = (int) alpha_idx;

		// ++histogramAlpha[vAlpha];




    g_double_floor_ring = false;
    g_filter_floor = false;
    g_gap_to_next_floor_ring = 35.0;

    if (floorFilter(g_filter_floor))
    {
      out_cloud_ptr->points.push_back(*it);

      vis_cloud_ptr->points.push_back(*it);
      vis_cloud_ptr->points.back().intensity = 1;


    }




  }
  // g_filter_name = "vanilla";
  // g_filter_name = "vanilla_0.45";
  g_filter_name = "show";




  // g_filter_name = "vanilla_" + to_string(g_min_retained_floor_radius) + "_" + to_string(g_max_retained_floor_radius);
  // if(g_double_floor_ring) {
  //   g_filter_name = "vanilla_" + to_string(g_min_retained_floor_radius) + "_" + to_string(g_max_retained_floor_radius) + "_" + to_string(g_gap_to_next_floor_ring);
  // }
  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  computeFilteredPointsData();
  // saveAlphaHistogram(histogramAlpha);
  updateROSParams();


}




////////////////////////////////////////////////////////////////////////////////

void
SCMapping::cylinderFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                PointCPtr &vis_cloud_ptr,
                                float x_axis_origin = 0,
                                float radius = 3,
                                float height = 100)
{
  // Formula for the Volume of a cylinder: M_PI * (radius^2) * height
  // Formula for the Radius of a cylinder: sqrt(Volume / (M_PI * height))
  // Formula for the Height of a cylinder: Volume / (M_PI * (radius^2))
  // Formula for the Diameter of a cylinder: (sqrt(Volume / (M_PI * height)))/2

  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();


  alpha_vec.clear();

  alpha_mean.data = 0.0;
  alpha_std.data = 0.0;
  alpha_min.data = 0.0;
  alpha_max.data = 0.0;




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

    // Transform the points to new frame
    transformPointCoordinates();
    

    if (floorFilter(g_filter_floor))
    {
      // If i need to start cylinder from the ground (as in a half cylinder), it will require a z axis origin of: g_robot_world_frame_coordinate.point.z
      // But i am interested in points in the direction of the lidar, and therefore this may not be the best option
      // Include this argument in the report.


      // the cylinder is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
      if ((!(((( (-height <= g_x) && (g_x <= -x_axis_origin) ) || ((x_axis_origin <= g_x) && (g_x <=  height)))) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(radius,2)))) || (g_z < g_floor_height)) // within the radius limits
          
      {
        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 1;
      }
      else
      {
        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.3;
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
  computeFilteredPointsData();
  updateROSParams();


}

////////////////////////////////////////////////////////////////////////////////

void
SCMapping::radiusFilter( PointCPtr &in_cloud_ptr,
                              PointCPtr &out_cloud_ptr,
                              PointCPtr &vis_cloud_ptr,
                              float min_radius = 0,
                              float max_radius = 250)
{

  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();


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

    // Transform the points to new frame
    transformPointCoordinates();
    

    if (floorFilter(g_filter_floor))
    {
      if (((( pow(g_x,2) + pow(g_y,2) ) >= pow(min_radius,2)) && (( pow(g_x,2) + pow(g_y,2) ) <= pow(max_radius,2))) || (g_z < g_floor_height))
      {
        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 1;
      }
      else
      {
        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.3;
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
  computeFilteredPointsData();
  updateROSParams();


}


////////////////////////////////////////////////////////////////////////////////

void
SCMapping::ringFilter( PointCPtr &in_cloud_ptr,
                                PointCPtr &out_cloud_ptr,
                                PointCPtr &vis_cloud_ptr,
                                float x_axis_origin = 15,
                                float ring_min_radius = 2,
                                float ring_max_radius = 40,
                                float ring_height = 50)
{
  // This ring filter will remove points in the inner cylinder and retain points within the outter cylinder as this combines certain properties of the cylinder and radius filters

  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();


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

    // Transform the points to new frame
    transformPointCoordinates();


    if (floorFilter(g_filter_floor))
    {
      // the ring is needed in both sides, front and back as the argument that the angle doesnt change in the line of motion still holds
      if (((!(( g_x >= (-x_axis_origin - ring_height) && g_x <= (x_axis_origin + ring_height)) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(ring_min_radius,2)))) && // within the min radius limits
          ((( g_x >= (-ring_height) && g_x <= (ring_height)) && // within the X limits
          (( pow(g_z,2) + pow(g_y,2) ) <= pow(ring_max_radius,2)))) ||
          (g_z < g_floor_height)) // within the max radius limits
        )
      {
        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 1;
      }
      else
      {
        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.3;
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
  computeFilteredPointsData();
  updateROSParams();


}

////////////////////////////////////////////////////////////////////////////////

void
SCMapping::boxFilter(PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr,
                          PointCPtr &vis_cloud_ptr,
                          float x_axis_min = -30, float x_axis_max = 30,
                          float y_axis_min = -3, float y_axis_max = 3,
                          float z_axis_min = -100, float z_axis_max = 100)
{
  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();
    
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

    // Transform the points to new frame
    transformPointCoordinates();

    if (floorFilter(g_filter_floor))
    {

      if ((!( ((g_z >= z_axis_min) && (g_z <= z_axis_max)) &&
            ((g_x >= x_axis_min) && (g_x <= x_axis_max)) &&
            ((g_y >= y_axis_min) && (g_y <= y_axis_max)))) || 
            (g_z < g_floor_height)) // Not within the box limits
      {
        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 1;
      }
      else
      {
        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.75;
      }
    }
    else
    {
      vis_cloud_ptr->points.push_back(*it);
      vis_cloud_ptr->points.back().intensity = 0.5;
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
  computeFilteredPointsData();
  updateROSParams();


}

////////////////////////////////////////////////////////////////////////////////

void
SCMapping::AlphaFilter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, PointCPtr &vis_cloud_ptr,
                            float min_angle = 0, float max_angle = 35)
{

  // Need to show fianl oerformance agianst vanilla visually, without saving metric data to show true performance and say how saving metrics have a small affect.

  // Doing this may not allow me to super impose filters, so need to think about how to go about this later
  out_cloud_ptr->points.clear();
  vis_cloud_ptr->points.clear();


  alpha_vec.clear();
  
  alpha_mean.data = 0.0;
  alpha_std.data = 0.0;
  alpha_min.data = 0.0;
  alpha_max.data = 0.0;

  g_matched_alpha = 0;



  ros::param::get("/previous_alpha_mean", previous_alpha_mean);
  ros::param::get("/previous_alpha_std", previous_alpha_std);
  ros::param::get("/previous_alpha_mean", previous_alpha_min);
  ros::param::get("/previous_alpha_max", previous_alpha_max);

  int previous_matched_alpha;
  ros::param::get("/previous_matched_alpha", previous_matched_alpha);

  min_angle = 0.75*previous_alpha_mean;
  max_angle = 0.9*previous_alpha_max;

  // min_angle = 0;
  // max_angle = 0.1;


  // cout << "Min Angle: " << endl;
  // cout << min_angle << endl;
  // cout << "Max Angle: " << endl;
  // cout << max_angle << endl;




  // Transform the points to new frame
  transformRobotCoordinates();

  g_previous_robot_world_frame_coordinate = g_robot_world_frame_coordinate;
  ros::param::get("/previous_robot_world_frame_coordinate_x", g_previous_robot_world_frame_coordinate.point.x);
  ros::param::get("/previous_robot_world_frame_coordinate_y", g_previous_robot_world_frame_coordinate.point.y);
  ros::param::get("/previous_robot_world_frame_coordinate_z", g_previous_robot_world_frame_coordinate.point.z);


  // Compute and save additional robot trajectory information to odometry
  computeTrajectoryInformation();

  // g_previous_robot_world_frame_coordinate

  // cout << "g_previous_robot_world_frame_coordinate: " << endl;
  // cout << g_previous_robot_world_frame_coordinate << endl;
  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {

    g_x = it->x;
    g_y = it->y;
    g_z = it->z;

    // Transform the points to new frame
    transformPointCoordinates();


    
    // Computing Observation Angle of point with respect to lidar frame:
    // double observation_angle = computeObservationAngle();


    // out_cloud_ptr->points.push_back(*it);

    // Computing Angle Deviation of point with respect to previous frame:
    double alpha = computeAlpha();
    //only pushing back the angle to the vector if the angle was computed properly, to enable computing correct statistics
    if((not isnan(alpha)))
    {
      alpha_vec.push_back(alpha);
      
    }


    if (floorFilter(g_filter_floor))
    {


      bool observable = computePointObservability();
      // bool observable = true;


      // Condition on the angle deviation on observed points.
      if(((not isnan(alpha)) && (alpha >= min_angle) && (alpha <= max_angle) && (observable)) || (g_z < g_floor_height)) {

        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 1;

        g_matched_alpha += 1;
        // cout << previous_matched_alpha << endl;

      }

      // else if (radiusCondition(g_x, g_y, g_z, 0.0, 15.0)) {

      //   out_cloud_ptr->points.push_back(*it);

      //   vis_cloud_ptr->points.push_back(*it);
      //   vis_cloud_ptr->points.back().intensity = 1;
      // }

      // else if ((g_z < floor_height) || (((( pow(g_x,2) + pow(g_y,2) ) >= pow(70,2)) || (( pow(g_x,2) + pow(g_y,2) ) <= pow(30,2)))
      // && cylinderCondition(g_x, g_y, g_z, 0, 4, 100))) {
      // else if ((g_z < floor_height) || ((( pow(g_x,2) + pow(g_y,2) ) >= pow(70,2)) || (( pow(g_x,2) + pow(g_y,2) ) <= pow(30,2)))) {

      //   out_cloud_ptr->points.push_back(*it);

      //   vis_cloud_ptr->points.push_back(*it);
      //   vis_cloud_ptr->points.back().intensity = 1;
      // }

      // else if ((!cylinderCondition(g_x, g_y, g_z, 60, 4, 120)) || radiusCondition(g_x, g_y, g_z, 60, 120) || (g_robot_angular_velocity > 5) || (g_robot_linear_velocity_abs < 0.4)) {

      //   out_cloud_ptr->points.push_back(*it);

      //   vis_cloud_ptr->points.push_back(*it);
      //   vis_cloud_ptr->points.back().intensity = 1;
      // }

      // else if ((!cylinderCondition(g_x, g_y, g_z, 0, 4, 120)) || radiusCondition(g_x, g_y, g_z, 60, 120) || (g_robot_angular_velocity > 5)) {

      //   out_cloud_ptr->points.push_back(*it);

      //   vis_cloud_ptr->points.push_back(*it);
      //   vis_cloud_ptr->points.back().intensity = 1;
      // }

      // else if (alpha_vec.size() < 100) { // To ensure that if the angle deviation cannot be computed for at least 100 points as well as the current point, include the point either way.
      
      // // To ensure that if the angle deviation is computed as 0 for the current point, include the point either way.
      // // This condition can signify that the robot has not trnslated or rotated at all, which may mean that the robot is still finding its initial pose.
      // else if (alpha == 0.0) { 

      // // To ensure that if the angle deviation is computed as 0 for the current point, include the point either way.
      // // This condition can signify that the robot has not trnslated or rotated at all, which may mean that the robot is still finding its initial pose.
      // // Also adding a condition on the number of points that suited the requirements of the angle constraint, to ensure a minimum number of points meet the requirements
      // // Can also create an adaptive threshold while this happens
      // else if ((alpha == 0.0) || (previous_matched_alpha < 7000)) { 
        
      // // To ensure that if the angle deviation is computed as 0 for the current point, include the point either way.
      // // This condition can signify that the robot has not trnslated or rotated at all, which may mean that the robot is still finding its initial pose.
      // // Also adding a condition on the number of points that suited the requirements of the angle constraint, to ensure a minimum number of points meet the requirements
      // // Can also create an adaptive threshold while this happens
      // // Also adding a condition that the current number of matched points need to be greater than a minimum threshold to account for unexpected changes in the current frame
      // else if ((alpha == 0.0) || (previous_matched_alpha < 7000) || (g_matched_alpha < 100)) { 

      // This condition can signify that the robot has not trnslated or rotated at all, which may mean that the robot is still finding its initial pose.
      // Also adding a condition on the number of points that suited the requirements of the angle constraint, to ensure a minimum number of points meet the requirements
      // Can also create an adaptive threshold while this happens
      // Also adding a condition that the rest of the points are sampled randomly with a 10% probability
      else if ( ((float) rand()/RAND_MAX) > 0.80) { 
        // cout <<"something is wrong" <<endl;
        out_cloud_ptr->points.push_back(*it);

        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.85;

      }

      // Also adding a condition that the current number of matched points need to be greater than a minimum threshold to account for unexpected changes in the current frame
      // else if ((g_matched_alpha < 10)) { 
      //   // cout <<"something is wrong" <<endl;
      //   out_cloud_ptr->points.push_back(*it);

      //   vis_cloud_ptr->points.push_back(*it);
      //   vis_cloud_ptr->points.back().intensity = 1;

      // }
      

      else // Condition to visualize unselected points with a different intensity
      {
        vis_cloud_ptr->points.push_back(*it);
        vis_cloud_ptr->points.back().intensity = 0.75;
      }

    }
    else if ( ((float) rand()/RAND_MAX) > 0.80) { 
      // cout <<"something is wrong" <<endl;
      out_cloud_ptr->points.push_back(*it);

      vis_cloud_ptr->points.push_back(*it);
      vis_cloud_ptr->points.back().intensity = 0.85;

    }
    else
    {
      vis_cloud_ptr->points.push_back(*it);
      vis_cloud_ptr->points.back().intensity = 0.5;
    }


  }



  computeAlphaStatistics();
  


  ros::param::get("/filter_name", g_filter_name);

  // if (g_filter_name.empty())
  // {
  //   g_filter_name = string("ang_dev") + string("_") + "0" + string("_") + "p1";
  // }
  // else if (g_filter_name.find("ang_dev") != string::npos)
  // {
  //   g_filter_name = g_filter_name;
  // }
  // else
  // {
  //   g_filter_name = g_filter_name + string("_") + string("ang_dev") + string("_") + "0" + string("_") + "p1";
  // }


  // ros::param::get("/filter_name", g_filter_name);

  // if (g_filter_name.empty())
  // {
  //   g_filter_name = string("ang_dev") + string("_") + "0.75mean" + string("_") + "0.9max";
  // }
  // else if (g_filter_name.find("ang_dev") != string::npos)
  // {
  //   g_filter_name = g_filter_name;
  // }
  // else
  // {
  //   g_filter_name = g_filter_name + string("_") + string("ang_dev") + string("_") + "0.75mean" + string("_") + "0.9max";
  // }

  // ros::param::get("/filter_name", g_filter_name);

  // if (g_filter_name.empty())
  // {
  //   g_filter_name = string("ang_dev") + string("_") + "p5mean" + string("_") + "max_cyl_60_4_120_rad_60_120_angularspeed_p4linearspeed_p01sampling";
  // }
  // else if (g_filter_name.find("ang_dev") != string::npos)
  // {
  //   g_filter_name = g_filter_name;
  // }
  // else
  // {
  //   g_filter_name = g_filter_name + string("_") + string("ang_dev") + string("_") + "p5mean" + string("_") + "max_cyl_60_4_120_rad_60_120_angularspeed_p4linearspeed_p01sampling";
  // }

  if (g_filter_name.empty())
  {
    g_filter_name = string("ang_dev") + string("_") + "show" + string("_") + "show";
  }
  else if (g_filter_name.find("ang_dev") != string::npos)
  {
    g_filter_name = g_filter_name;
  }
  else
  {
    g_filter_name = g_filter_name + string("_") + string("ang_dev") + string("_") + "show" + string("_") + "show";
  }
  
  

  g_in_cloud_size = in_cloud_ptr->size();
  g_out_cloud_size = out_cloud_ptr->size();
  computeFilteredPointsData();
  updateROSParams();


}



////////////////////////////////////////////////////////////////////////////////

void
SCMapping::Add(PointCPtr &cloud1, PointCPtr &cloud2)
{
  for ( PointC::iterator it = cloud1->begin(); it != cloud1->end(); it++)
  {
    cloud2->points.push_back(*it);
  } 
  
}


////////////////////////////////////////////////////////////////////////////////

void
SCMapping::callback(const sensor_msgs::PointCloud2ConstPtr& filtered_cloud_msg)
// SCMapping::callback(const sensor_msgs::PointCloud2ConstPtr& filtered_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& floor_cloud_msg)
{// declare type
  PointCPtr filtered_cloud(new PointC);
  // PointCPtr floor_cloud(new PointC);
  PointCPtr cloud1(new PointC);
  PointCPtr cloud2(new PointC);
  PointCPtr cloud3(new PointC);

  PointCPtr cloud_out(new PointC);
  PointCPtr vis_cloud(new PointC);

  // These are the two synced clouds we are subscribing to, however both of these are currently same
  pcl::fromROSMsg(*filtered_cloud_msg, *filtered_cloud);
  // pcl::fromROSMsg(*floor_cloud_msg, *floor_cloud);
  //////////////////////////////////////////////////

  // Bool to determine weather to filter the floor.
  // Need to check how many more points apart from the floor are filtered by my filters
  g_filter_floor = true;


  // g_min_retained_floor_radius = 0;
  // g_min_retained_floor_radius = 5;
  // g_min_retained_floor_radius = 10;
  // g_min_retained_floor_radius = 15;
  // g_min_retained_floor_radius = 20;
  g_min_retained_floor_radius = 10;
  // g_min_retained_floor_radius = 30;
  // g_min_retained_floor_radius = 50;
  // g_min_retained_floor_radius = 70;
  // g_max_retained_floor_radius = 10;
  // g_max_retained_floor_radius = 15;
  // g_max_retained_floor_radius = 20;
  // g_max_retained_floor_radius = 25;
  // g_max_retained_floor_radius = 1000;
  // g_max_retained_floor_radius = 30;
  g_max_retained_floor_radius = 20;
  // g_max_retained_floor_radius = 40;
  // g_max_retained_floor_radius = 60;
  // g_max_retained_floor_radius = 80;

  // Filter(filtered_cloud, cloud_out, vis_cloud); //removed suspected unneccesary points

  // Explain the naming convension of the test files properly in the thesis.

  // Floor Removal Tests --> Try these with and without removing floor
  // Filter(filtered_cloud, cloud_out, vis_cloud); //removed suspected unneccesary points
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 0, 50); //removed suspected unneccesary points in form of radius filter
  // ringFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 50, 100); //removed suspected unneccesary points in form of ring filter

  // Cylinder Filters
  // To test best thickness of forward points to be eliminated
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 2, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 4, 100); //removed suspected unneccesary points in form of cylinder filter --> works as the best cyliner filter with almost 45% of filtered cloud
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 5, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 6, 100); //removed suspected unneccesary points in form of cylinder filter --> note in report that 6 fails

  // To test range of points required to be removed
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 10); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 20); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 30); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 40); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 0, 3, 50); //removed suspected unneccesary points in form of cylinder filter

  // To test where to start the cylinder filter from
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 2, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 5, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 10, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 15, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 20, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 30, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 40, 3, 100); //removed suspected unneccesary points in form of cylinder filter

  // To test best range of forward points to be eliminated
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 80, 3, 100); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 60, 3, 80); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 40, 3, 60); //removed suspected unneccesary points in form of cylinder filter
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 20, 3, 40); //removed suspected unneccesary points in form of cylinder filter
  // The height of the filter here should be based on results here and the test from the range evaluations, so i can add them here
  // 0 to 20 is already tested previously, use it again here


  // Trying cylinder parameters that worked quite well individually and comparing performance against all other cylinder filters.
  // cylinderFilter(filtered_cloud, cloud_out, vis_cloud, 15, 4, 80); //removed suspected unneccesary points in form of cylinder filter
  // Need to run the evaluation of radius being 5 again


  // Radius Filters
  // To test inner radius of points required to be removed
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 0, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 2, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 4, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 6, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 8, 50); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 10, 50); //removed suspected unneccesary points in form of radius filter

  // To test outer radius of points required to be retained
  // USe the inner radius herre based on the best inner radius identified and add some more tests to this
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 0, 40); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 0, 30); //removed suspected unneccesary points in form of radius filter
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 0, 20); //removed suspected unneccesary points in form of radius filter

  // Trying min and max radius parameters that worked quite well individually and comparing performance against all other radius filters.
  // radiusFilter(filtered_cloud, cloud_out, vis_cloud, 8, 30); //removed suspected unneccesary points in form of radius filter


  // Angle Deviation Filters
  // To test inner radius of points required to be removed
  // AlphaFilter(filtered_cloud, cloud_out, vis_cloud, 0, 30); //removed suspected unneccesary points in form of angle deviation filter
  AlphaFilter(filtered_cloud, cloud_out, vis_cloud); //removed suspected unneccesary points in form of angle deviation filter


  // Ring Filters // Test based on best height range from cylinder filter, range from radius filter, and inner radius of cylinder 
  // ringFilter(filtered_cloud, cloud_out, vis_cloud, 15, 2, 40, 50); //removed suspected unneccesary points in form of ring filter


  // Box Filters // Test based on best height range from cylinder filter, range from radius filter, and inner radius of cylinder 
  // boxFilter(filtered_cloud, cloud_out, vis_cloud, -30, 30, -3, 3, -100, 100); //removed suspected unneccesary points in form of ring filter
  


  // write head and publish output
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 vis_output;
  
  pcl::toROSMsg(*cloud_out, output);
  pcl::toROSMsg(*vis_cloud, vis_output);
  output.header = filtered_cloud_msg->header;
  vis_output.header = filtered_cloud_msg->header;
  pub_.publish (output);
  pub_vis_selected_points_.publish (vis_output);
}

////////////////////////////////////////////////////////////////////////////////

void
SCMapping::odom_callback(const nav_msgs::OdometryConstPtr& odom_in)
{
  
  // Need to use this to predict where an observed point was in the previous frame

  nav_msgs::Odometry robot_odom;
  robot_odom = *odom_in;


  // Converting Odom Orientation from Quaternion to Roll Pitch and Yaw:
  double q_x = robot_odom.pose.pose.orientation.x;
  double q_y = robot_odom.pose.pose.orientation.y;
  double q_z = robot_odom.pose.pose.orientation.z;
  double q_w = robot_odom.pose.pose.orientation.w;

  // Quaternion
  tf2::Quaternion q(q_x, q_y, q_z, q_w);
  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);
  // Roll Pitch and Yaw from rotation matrix
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);


  g_robot_angle = abs(yaw * 180 / M_PI);

  ros::param::get("/robot_previous_angle", g_robot_previous_angle);

  if ((abs(g_robot_previous_angle) > 175) && (abs(g_robot_angle) < 0.5)) { //Can probably reduce this from 0.5
    g_robot_angle = 180;
  }

      


}


////////////////////////////////////////////////////////////////////////////////

void
SCMapping::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_in)
{

  g_robot_imu = *imu_in;

  g_robot_orientation = g_robot_imu.orientation;

  // g_robot_angular_velocity = g_robot_imu.angular_velocity;
  // ros::param::set("/robot_angular_velocity_x", g_robot_angular_velocity.x);
  // ros::param::set("/robot_angular_velocity_y", g_robot_angular_velocity.y);
  // ros::param::set("/robot_angular_velocity_z", g_robot_angular_velocity.z);

  g_robot_linear_acceleration = g_robot_imu.linear_acceleration;
  ros::param::set("/robot_linear_acceleration_x", g_robot_linear_acceleration.x);
  ros::param::set("/robot_linear_acceleration_y", g_robot_linear_acceleration.y);
  ros::param::set("/robot_linear_acceleration_z", g_robot_linear_acceleration.z);



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

  ros::init (argc, argv, "selection_criteria_mapping");

  /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");

  // Create a selection criteria mapping object
  SCMapping SCLObject (nh);
  
   
  ros::spin ();
  
  return (0);

} 