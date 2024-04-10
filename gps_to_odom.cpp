#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void pubCallback( const std_msgs::String::ConstPtr& msg){

    ROS_INFO("sento : [%s]", msg->data.c_str());

}

int main(int argc , char** argv){

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/fix", 10, pubCallback);
    ros::spin();

    return 0;
}

