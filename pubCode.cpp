#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <cmath>

double calculateDirection(double x1, double y1, double x2, double y2) {
    // Calcola l'angolo in radianti tra due punti (x1, y1) e (x2, y2)
    return atan2(y2 - y1, x2 - x1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_odom_publisher");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);

    ros::Rate loop_rate(10);  // 10 Hz

    double prev_e = 0.0;
    double prev_n = 0.0;

    while (ros::ok()) {
        // Replace these values with your ENU coordinates
        double e = 10.0;
        double n = 5.0;
        double u = 0.0;  // Assuming flat terrain

        // Calculate direction (angle) between previous and current position
        double direction = calculateDirection(prev_e, prev_n, e, n);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Position
        odom.pose.pose.position.x = e;
        odom.pose.pose.position.y = n;
        odom.pose.pose.position.z = u;

        // Orientation (quaternion)
        double theta = direction;  // Angle in radians
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(theta / 2);
        odom.pose.pose.orientation.w = cos(theta / 2);

        // Twist (not necessary for your case, but included for completeness)
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        // Publish the message
        odom_pub.publish(odom);

        // Update previous position
        prev_e = e;
        prev_n = n;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
