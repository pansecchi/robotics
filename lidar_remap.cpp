#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parameterConfig.h>

std::string frame_set = "wheel_odom"; // Valore predefinito
ros::Publisher pub_lidar;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    // Crea un oggetto del tipo sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 point_cloud_msg = *msg; 
    point_cloud_msg.header.frame_id = frame_set;
    pub_lidar.publish(point_cloud_msg);
}

void callback(first_project::parameterConfig &config, uint32_t level) {

    if (config.frame_set == 1) {
        frame_set = "wheel_odom";

    } else if (config.frame_set == 2) {
        frame_set = "gps_odom";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_remap");
    // Subscribe to odometry message
    ros::NodeHandle n;

    ros::Subscriber sub_lidar = n.subscribe("/os_cloud_node/points", 10, lidarCallback);
    pub_lidar = n.advertise<sensor_msgs::PointCloud2>("PointCloud_remapped",10);

    dynamic_reconfigure::Server<first_project::parameterConfig> server;
    dynamic_reconfigure::Server<first_project::parameterConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();

    return 0;

}
