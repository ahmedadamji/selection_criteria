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

#include <selection_criteria/remove_floor.h>

// NAMESPACES
using namespace std;
using namespace message_filters;


// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;



////////////////////////////////////////////////////////////////////////////////
// Will return points based on the conditions set by the algorithm
// CURRENTLY USING HDL_GRAPH_SLAM FILTERING FUNCTIONALITIES, REFERENCE THIS IF FINAL!
// IT IS SLIGHTLY MODIFIED TO NOT INCLUDE FLOOR POINTS
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
RmFloor::RmFloor (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  debug_ (false)
{
  nh_ = nh;

  // Define the publishers
  pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/filtered_points_without_floor", 3, true);
  

  // // Create a ROS subscriber for the input point cloud
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c1(nh_, "/filtered_points", 1); //check if anything is even published to filtered points
  // message_filters::Subscriber<sensor_msgs::PointCloud2> c2(nh_, "/filtered_points", 1);
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), c1, c2);
  
  // sync.registerCallback(boost::bind(&RmFloor::callback, this, _1, _2));


  // Create a ROS subscriber for the input point cloud
  sub_ = nh_.subscribe("/filtered_points", 3, &RmFloor::callback, this);

  // tilt_deg = nh_.param<double>("tilt_deg", 0.0);
  // sensor_height = nh_.param<double>("sensor_height", 2.0);
  // height_clip_range = nh_.param<double>("height_clip_range", 1.0);
  // floor_pts_thresh = nh_.param<int>("floor_pts_thresh", 512);
  // floor_normal_thresh = nh_.param<double>("floor_normal_thresh", 10.0);
  // use_normal_filtering = nh_.param<bool>("use_normal_filtering", true);
  // normal_filter_thresh = nh_.param<double>("normal_filter_thresh", 20.0);

}


////////////////////////////////////////////////////////////////////////////////

void
RmFloor::Filter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  out_cloud_ptr->points.clear();

  
  for ( PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    //out_cloud_ptr->points.push_back(*it);


    g_x = it->x;
    g_y = it->y;
    g_z = it->z;
    
    // This is a floor condition where points above the floor are selected based on height, this has not been yet implemented in a function or a filter.
    // Can I try to filter the points that have been detected as floor points by the floor detection nodelet.
    // And is there any reason for me to do this?
    if ( g_z >= 0.05)
    {
      out_cloud_ptr->points.push_back(*it);
    }


  }


}


////////////////////////////////////////////////////////////////////////////////

void
RmFloor::Add(PointCPtr &cloud1, PointCPtr &cloud2)
{
  for ( PointC::iterator it = cloud1->begin(); it != cloud1->end(); it++)
  {
    cloud2->points.push_back(*it);
  } 
  
}

////////////////////////////////////////////////////////////////////////////////


void
RmFloor::detect(PointCPtr &cloud1) {
  // compensate the tilt rotation
  Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
  tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

  // filtering before RANSAC (height and normal filtering)
  PointCPtr filtered(new PointC);
  pcl::transformPointCloud(*cloud1, *filtered, tilt_matrix);
  // filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
  // filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

  if(use_normal_filtering) {
    // filtered = normal_detection(filtered);
  }

  pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

  // too few points for RANSAC
  if(filtered->size() < floor_pts_thresh) {
    return;
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
  pcl::RandomSampleConsensus<PointT> ransac(model_p);
  ransac.setDistanceThreshold(0.1);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.getInliers(inliers->indices);

  // too few inliers
  if(inliers->indices.size() < floor_pts_thresh) {
    return;
  }

  // verticality check of the detected floor's normal
  Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  double dot = coeffs.head<3>().dot(reference.head<3>());
  if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
    // the normal is not vertical
    return;
  }

  PointCPtr inlier_cloud(new PointC);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(filtered);
  extract.setIndices(inliers);
  extract.filter(*inlier_cloud);
  inlier_cloud->header = cloud1->header;

  // for (std::vector<pcl::PointIndices>::const_iterator it = inliers.begin (); it != inliers.end (); ++it)
  // {

  // }


}


////////////////////////////////////////////////////////////////////////////////


void
RmFloor::plane_clip(PointCPtr& src_cloud, PointCPtr &cloud2, const Eigen::Vector4f& plane, bool negative) {
  pcl::PlaneClipper3D<PointT> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  PointCPtr dst_cloud(new PointC);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  // return dst_cloud;
}


////////////////////////////////////////////////////////////////////////////////


void
RmFloor::normal_detection(PointCPtr& cloud, PointCPtr &cloud2) {
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(10);
  ne.setViewPoint(0.0f, 0.0f, sensor_height);
  ne.compute(*normals);

  PointCPtr normal_points(new PointC);
  normal_points->reserve(cloud->size());

  for(int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
    if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
      normal_points->push_back(cloud->at(i));
    }
  }

  normal_points->width = normal_points->size();
  normal_points->height = 1;
  normal_points->is_dense = false;

  cloud2 = normal_points;
}

////////////////////////////////////////////////////////////////////////////////

void
RmFloor::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{// declare type
  PointCPtr cloud1f(new PointC);
  PointCPtr cloud1(new PointC);
  PointCPtr cloud2(new PointC);
  PointCPtr cloud_all(new PointC);

  // These are the two synced clouds we are subscribing to, however both of these are currently same
  pcl::fromROSMsg(*cloud_msg1, *cloud1f);
  //pcl::fromROSMsg(*cloud_msg2, *cloud2);
  //////////////////////////////////////////////////

  Filter(cloud1f, cloud_all); //removed suspected unneccesary points

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

  ros::init (argc, argv, "remove_floor");

  /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");

  // Create a selection criteria localization object
  RmFloor SCLObject (nh);
  
   
  ros::spin ();
  
  return (0);

} 