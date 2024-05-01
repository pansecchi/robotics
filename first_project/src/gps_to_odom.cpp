#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"

struct GPSPoint {
    double latitude;  // in degrees
    double longitude; // in degrees
    double altitude;  // in meters
};

struct ECEFPoint {
    double x; 
    double y; 
    double z; 
};

struct ENUPoint {
    double e;
    double n;
    double u;
};

ENUPoint prev_enu{0,0,0};

double refLat = 0.00;   
double refLon = 0.00;    
double refAlt = 0.00;

const double a = 6378137.0;     // Semi-major axis of Earth ellipsoid in meters
const double f = 1 / 298.257223; // Flattening
const double b = a * (1 - f);   // Semi-minor axis

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

ECEFPoint convertGPSToECEF(const GPSPoint& gps) {
    double latitude = toRadians(gps.latitude);
    double longitude = toRadians(gps.longitude);

    double N = a / sqrt(1 - pow(sin(latitude), 2) * pow(f, 2));

    ECEFPoint ecef;
    ecef.x = (N + gps.altitude) * cos(latitude) * cos(longitude);
    ecef.y = (N + gps.altitude) * cos(latitude) * sin(longitude);
    ecef.z = (N * (1 - pow(f, 2)) + gps.altitude) * sin(latitude);

    return ecef;
}

ENUPoint convertECEFToENU(const ECEFPoint& ecef) {
    double refLatRad = toRadians(refLat);
    double refLonRad = toRadians(refLon);
    

    double sinLat = sin(refLatRad);
    double cosLat = cos(refLatRad);
    double sinLon = sin(refLonRad);
    double cosLon = cos(refLonRad);

    ENUPoint enu;

    double dx = ecef.x - refAlt * cosLat * cosLon;
    double dy = ecef.y - refAlt * cosLat * sinLon;
    double dz = ecef.z - refAlt * sinLat;

    enu.e = -sinLon * dx + cosLon * dy;
    enu.n = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    enu.u = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

    return enu;
}

double calculateDirection(double x1, double y1, double x2, double y2) {

    return atan2(y2 - y1, x2 - x1);
    
}

ros::Publisher odom_pub;

void Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

    GPSPoint gps{msg->latitude, msg->longitude, msg->altitude};

    // Convert GPS coordinates to ECEF
    ECEFPoint ecef = convertGPSToECEF(gps);

    // convert ECEF to ENU
    ENUPoint enu = convertECEFToENU(ecef);

    //traslation correction
    double angle = 2.235;
    enu.e = enu.e - 45.444;
    enu.n = enu.n - 15.302;

    //rotation correction
    double xTrasl = (enu.e * cos(angle) - enu.n * sin(angle));
    double yTrasl = (enu.e * sin(angle) + enu.n * cos(angle));
    enu.e = xTrasl;
    enu.n = yTrasl;

    double direction = calculateDirection(prev_enu.e, prev_enu.n, enu.e, enu.n);

    
    nav_msgs::Odometry odom;

    // Header informations
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    // Position
    odom.pose.pose.position.x = enu.e;
    odom.pose.pose.position.y = enu.n;
    odom.pose.pose.position.z = 0.0;

    // Orientation (quaternion)
    double theta = direction;  
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
    
    prev_enu = enu;

}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n;

    // Load parameters from launch file
    n.getParam("/reflat", refLat);
    n.getParam("/reflon", refLon);
    n.getParam("/refalt", refAlt);
    
    ros::Subscriber gps_to_odom = n.subscribe("/fix", 10, Callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10);
    ros::spin();

    return 0;

}

