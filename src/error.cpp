#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
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
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define RAD2DEG 57.295779513

// Added:
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using namespace message_filters;

typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<std_msgs::String>("/slam_error/ape", 10);
  }
  void gotodomCallback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2)
{
geometry_msgs::TransformStamped  msg1g, msg2g;
msg1g.header.stamp = msg1->header.stamp;
msg1g.header.frame_id = "psa_map";
msg1g.child_frame_id = "base_link";
msg1g.transform.rotation.x = msg1->pose.pose.orientation.x;
msg1g.transform.rotation.y = msg1->pose.pose.orientation.y;
msg1g.transform.rotation.z = msg1->pose.pose.orientation.z;
msg1g.transform.rotation.w = msg1->pose.pose.orientation.w;
msg1g.transform.translation.x = msg1->pose.pose.position.x;
msg1g.transform.translation.y = msg1->pose.pose.position.y;
msg1g.transform.translation.z = msg1->pose.pose.position.z;

msg2g.header.stamp = msg1->header.stamp;
msg2g.header.frame_id = "psa_map";
msg2g.child_frame_id = "base_link";
msg2g.transform.rotation.x = msg2->pose.pose.orientation.x;
msg2g.transform.rotation.y = msg2->pose.pose.orientation.y;
msg2g.transform.rotation.z = msg2->pose.pose.orientation.z;
msg2g.transform.rotation.w = msg2->pose.pose.orientation.w;
msg2g.transform.translation.x = msg2->pose.pose.position.x;
msg2g.transform.translation.y = msg2->pose.pose.position.y;
msg2g.transform.translation.z = msg2->pose.pose.position.z;
ros::Duration cache_(10);
tf2_ros::Buffer tfBuffer(cache_);
tf2_ros::TransformListener tfListener(tfBuffer);

geometry_msgs::TransformStamped geo_msg, msg11, msg22;
try{
      geo_msg = tfBuffer.lookupTransform("base_link", "psa_map", ros::Time(0), ros::Duration(3.0));
      
      tf2::doTransform(msg1g, msg11, geo_msg);
      tf2::doTransform(msg2g, msg22, geo_msg);
   }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
         
    }
//////////////////////
// compute longitudinal shift x relative to base_link
  double x = msg11.transform.translation.x - msg22.transform.translation.x;
// compute lateral shift  y relative to base_link
  double y = msg11.transform.translation.y - msg22.transform.translation.y;
// compute difference in orientation
    tf2::Quaternion q1(
        msg1->pose.pose.orientation.x,
        msg1->pose.pose.orientation.y,
        msg1->pose.pose.orientation.z,
        msg1->pose.pose.orientation.w);
    tf2::Quaternion q2(
        msg2->pose.pose.orientation.x,
        msg2->pose.pose.orientation.y,
        msg2->pose.pose.orientation.z,
        msg2->pose.pose.orientation.w);
    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m1(q1);
    tf2::Matrix3x3 m2(q2);
    // Roll Pitch and Yaw from rotation matrix
    double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    m1.getRPY(roll1, pitch1, yaw1);
    m2.getRPY(roll2, pitch2, yaw2);
  double APE = sqrt(x*x + y*y);
 double RPE = fabs( (yaw1-yaw2) * RAD2DEG);
 if(RPE>300) {RPE=fabs(RPE-360);}
// Output the measure
   ROS_INFO("xg_map: %.2f, yg_map: %.2f, shift_longitudinal: %.2f, shift_lateral: %.2f APE: %.2f, RPE: %.2f",msg1->pose.pose.position.x,
    msg1->pose.pose.position.y,x,y,  APE, RPE);
       std_msgs::String msg;
       std::stringstream ss;
       ss << msg1->pose.pose.position.x<< "  " << msg1->pose.pose.position.y<< "  "<< x << " "<< y<<"  " <<APE <<"   "<< RPE;
       msg.data = ss.str();
   pub_.publish(msg);
}
 private:
  ros::NodeHandle n_; 
  ros::Publisher pub_; 

};

// Function for rotation matrix to quaternion conversion:
std::vector<double> rotmat2q(double T[3][3]) {

  std::vector<double> q;

  int tr = 0;
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) {
      if(i == j) {
        tr = tr + T[i][j];
      }
    }
  }

  // This is the wrong way to set values
  if (tr == 4) {
    q.push_back(1.0);
    q.push_back(0.0);
    q.push_back(0.0);
    q.push_back(0.0);
    return q;
  }

  double angle = acos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2);

  xr = T[2, 1] - T[1, 2];
  yr = T[0, 2] - T[2, 0];
  zr = T[1, 0] - T[0, 1];

  x = xr/sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2));
  y = yr/sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2));
  z = zr/sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2));

  q.push_back(cos(angle/2));
  q.push_back(x * sin(angle/2));
  q.push_back(sin(angle/2));
  q.push_back(z * sin(angle/2));

  return q;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "error");
	  
  ros::NodeHandle n;

  message_filters::Subscriber<nav_msgs::Odometry> sub_gt(n, "/aide/odometry/filtered/psamap/baselink", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "/odom", 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub_gt, sub_odom);
  
  SubscribeAndPublish SAPObject;
  
  sync.registerCallback(boost::bind(&SubscribeAndPublish::gotodomCallback, &SAPObject, _1, _2));

  ros::spin();

  
  return 0;
}
