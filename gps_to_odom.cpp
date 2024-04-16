#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
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

ENUPoint prev_enu{45.441519,15.122844,-17.458665};
double heading;

// Reference point (latitude, longitude, altitude)
double refLat = 45.477210;   
double refLon = 9.226158;    
double refAlt = 168.276000+ 6378137.0;



const double a = 6378137.0;     // Semi-major axis of Earth ellipsoid in meters
const double f = 1 / 298.257223; // Flattening
const double b = a * (1 - f);   // Semi-minor axis

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert GPS coordinates to ECEF coordinates
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


void Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

    
    GPSPoint gps{msg->latitude, msg->longitude, msg->altitude};

    // Convert GPS coordinates to ECEF
    ECEFPoint ecef = convertGPSToECEF(gps);
    
    //ROS_INFO("x: %f y: %f z:%f ", ecef.x , ecef.y , ecef.z);

    ENUPoint enu = convertECEFToENU(ecef);

    

    heading = atan( ( enu.n - prev_enu.n ) / ( enu.e - prev_enu.e ) );
    ROS_INFO("e: %f n: %f u:%f h: %f", enu.e , enu.n , enu.u , heading);
    prev_enu = enu;

}

int main(int argc, char **argv){

        
        ros::init(argc, argv, "gps_to_odom");
        ros::NodeHandle n;
        ros::Subscriber gps_to_odom = n.subscribe("/fix", 10, Callback);
        ros::spin();

        return 0;

}