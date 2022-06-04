#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <rosbag/bag.h>

rosbag::Bag bag;

void callback(nav_msgs::Odometry msg)
{
    ros::Time elapsed_time = ros::Time::now() - pre_publish_time_;
    bag.write("odom_transformed", elapsed_time, msg);
}

int main(int argc, char** argv)
{

    // However, you can have a rough estimation using pre_publish_time_ = ros::Time::now() just before the publish() call 
    // and computes the elapsed time in the subscriber callback as ros::Time elapsed_time = ros::Time::now() - pre_publish_time_, 
    // where pre_publish_time_ is a ros::Time member variable.



    ros::init(argc, argv, "record_bag");

    pre_publish_time_ = ros::Time::now();
	  
    ros::NodeHandle nh;
    

    bag.open("odom_transformed.bag", rosbag::bagmode::Write);
    
    ros::Subscriber sub = nh.subscribe("odom_transformed", 1000, callback);


    ros::spin();

    bag.close();
    
    return 0;
}
