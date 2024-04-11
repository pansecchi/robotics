#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "math.h"

// Definizione della costante pi greco
const double PI = 3.14159265358979323846;

double latitude;
double longitude;
double altitude;
double A = 6378000;  //equatorial earth radius
double B = 6357000;  //polar earth radius
float E = 0.0167;   //earth eccentricity
double ecef[3];     // ECEF  cartesian coordinates wrt earth centre
double ref_ecef[3] = {-100889.434334, -33666.072352, 6336240.767261};

void ECEF_to_ENU(const double ecef[3], const double ref_ecef[3], double enu[3]) {

    // Calcola la latitudine e la longitudine del punto di riferimento
    double latit = std::atan2(ref_ecef[2], std::sqrt(ref_ecef[0] * ref_ecef[0] + ref_ecef[1] * ref_ecef[1]));
    double longit = std::atan2(ref_ecef[1], ref_ecef[0]);

    // Calcola la matrice di rotazione da ECEF a ENU
    double rotation_matrix[3][3] = {
        {-std::sin(longit), std::cos(longit), 0},
        {-std::sin(latit) * std::cos(longit), -std::sin(latit) * std::sin(longit), std::cos(latit)},
        {std::cos(latit) * std::cos(longit), std::cos(latit) * std::sin(longit), std::sin(latit)}
    };

    // Calcola il vettore di traslazione da ECEF a ENU
    double translation_vector[3] = {-ref_ecef[0], -ref_ecef[1], -ref_ecef[2]};

    // Moltiplica le coordinate ECEF del punto di interesse per la matrice di rotazione e aggiungi il vettore di traslazione
    for (int i = 0; i < 3; ++i) {
        enu[i] = 0;
        for (int j = 0; j < 3; ++j) {
            enu[i] += rotation_matrix[i][j] * (ecef[j] + translation_vector[j]);
        }
    }
}





void Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

    //importing variables (latitude and longitude converted in degrees)
    latitude = (msg->latitude) * 2.0 * PI /180.0;
    longitude = (msg->longitude) * 2.0 * PI /180.0;
    altitude = (msg->altitude);
    //ROS_INFO("Latitude: %f, Longitude: %f, Altitude: %f ", latitude, longitude, altitude);


    //computation of the local earth radius
    double N = A / sqrt(1 - (E * E) * sin(latitude) * sin(latitude));


    //GPS to ECEF
    ecef[0] = (N + altitude) * cos(latitude) * cos(longitude);
    ecef[1] = (N + altitude) * cos(latitude) * sin(longitude);
    ecef[2] = (N * ((B * B) / (A * A)) + altitude) * sin(latitude);

    double temp = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1] + ecef[2] * ecef[2]);
    ROS_INFO("radius from calculation: %f", temp);
}




int main(int argc, char** argv) {

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n;
    ros::Subscriber gps_to_odom = n.subscribe("/fix", 10, Callback);
    ros::spin();

    return 0;

}
