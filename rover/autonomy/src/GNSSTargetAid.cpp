#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


typedef struct localCoord {
    double x;
    double y;
} localCoord;

class GNSSTargetAid {
public:
    double ENx;
    double ENy;
    double targetLat;
    double targetLon;
    double frameBearing;
    double curLat;
    double curLon;
    double localx;
    double localy;
    double offset = NULL;
    double initial_heading = 0;
    GNSSTargetAid(double lat, double lon, double bearing);
    GNSSTargetAid();
    localCoord getGNSSLocalization(double lat, double lon);
    void GNSSLocalizationProcess(double lat, double lon);
    double getDistance(localCoord c);
    double getBearing(localCoord c);
    double getDistance();
    double getBearing();
    void setTarget(double lat, double lon);
    void setBearing(double bearing);
    void setBearingFromGyro(double bearing);
    // double getBearing(double lat, double lon);
};

GNSSTargetAid::GNSSTargetAid(double lat, double lon, double bearing) {
    targetLat = lat;
    targetLon = lon;
    frameBearing = bearing * M_PI / 180.0;
}

GNSSTargetAid::GNSSTargetAid() {
    targetLat = 0;
    targetLon = 0;
    frameBearing = 0;
}

void GNSSTargetAid::GNSSLocalizationProcess(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    double phim = (lat + targetLat) / 2 * M_PI / 180.0;
    ENy = (targetLat - lat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (targetLon - lon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
    localy = ENy * cos(frameBearing) + ENx * sin(frameBearing);
    localx = ENx * cos(frameBearing) - ENy * sin(frameBearing);
}

localCoord GNSSTargetAid::getGNSSLocalization(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    localCoord coord;
    double phim = (lat + targetLat) / 2 * M_PI / 180.0;
    ENy = (targetLat - lat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (targetLon - lon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
    coord.y = ENy * cos(frameBearing) + ENx * sin(frameBearing);
    coord.x = ENx * cos(frameBearing) - ENy * sin(frameBearing);
    localx = coord.x;
    localy = coord.y;
    return coord;
}

double GNSSTargetAid::getBearing(localCoord c) {
    return atan2(c.y, c.x) * 180.0 / M_PI;
}

double GNSSTargetAid::getBearing() {
    return atan2(localy, localx) * 180.0 / M_PI;
}

double GNSSTargetAid::getDistance() {
    return sqrt(pow(localx, 2) + pow(localy, 2));
}

void GNSSTargetAid::setTarget(double lat, double lon) {
    targetLat = lat;
    targetLon = lon;
}

void GNSSTargetAid::setBearing(double bearing) {
    frameBearing = bearing * M_PI / 180.0;
}

void GNSSTargetAid::setBearingFromGyro(double bearing) {
    frameBearing = bearing * M_PI / 180.0 + offset;
}

GNSSTargetAid gnss = GNSSTargetAid();

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gnss.GNSSLocalizationProcess(msg->latitude, msg->longitude);
    ROS_INFO("targetLat: %f, targetLon: %f", gnss.targetLat, gnss.targetLon);
    ROS_INFO("curLat: %f, curLon: %f", gnss.curLat, gnss.curLon);
    ROS_INFO("move East by [m]: %f, move North by [m]: %f", gnss.ENx, gnss.ENy);
    ROS_INFO("x(to move to the right by [m]): %f, y(to move forward by [m]): %f", gnss.localx, gnss.localy);
    localCoord glo = {gnss.ENx, gnss.ENy};
    ROS_INFO("distance to target [m]: %f, bearing to target [degree]: %f, global bearing to target [degree] %f", gnss.getDistance(), gnss.getBearing(), gnss.getBearing({glo}));
    ROS_INFO("curr bearing %f", gnss.frameBearing);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    gnss.setBearing(atan2(msg->magnetic_field.y, -msg->magnetic_field.x) * 180.0 / M_PI);
}

void targetCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gnss.setTarget(msg->latitude, msg->longitude);
}

void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    Quaternion q;
    q.w = msg->orientation.w;
    q.x = msg->orientation.x;
    q.y = msg->orientation.y;
    q.z = msg->orientation.z;
    EulerAngles e = ToEulerAngles(q);
    if (gnss.offset == NULL) {
        gnss.offset = - e.yaw - gnss.initial_heading;
    } else {
        gnss.setBearingFromGyro(e.yaw * 180.0 / M_PI);
    }
}

int main(int argc, char **argv) {
    // gnss.setTarget(43.139199, -79.114206);
    // gnss.setBearing(0);
    ros::init(argc, argv, "GNSSLocalization");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ublox/fix", 1000, chatterCallback);
    // ros::Subscriber subm = n.subscribe("/zed2i/zed_node/imu/mag", 10, magCallback);
    ros::Subscriber subg = n.subscribe("/zed2i/zed_node/imu/data", 10, gyroCallback);
    ros::Subscriber subt = n.subscribe("/gps_target", 10, targetCallback);
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("GPSOdometry", 1000);
    ros::Rate loop_rate(10);
    // GNSSTargetAid gnss(43.139199, -79.114206, 0);
    while (ros::ok()) {
        ros::spinOnce();
        nav_msgs::Odometry msg;
        msg.pose.pose.position.x = gnss.localx;
        msg.pose.pose.position.y = gnss.localy;
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}