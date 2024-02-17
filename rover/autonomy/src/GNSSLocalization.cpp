#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace std;
GNSSLocalization gnss;

typedef struct localCoord {
    double x;
    double y;
} localCoord;

class GNSSLocalization {
public:
    double baseLat;
    double baseLon;
    double curLat;
    double curLon;
    double localx;
    double localy;
    GNSSLocalization(double lat, double lon);
    localCoord getGNSSLocalization(double lat, double lon);
    void GNSSLocalizationProcess(double lat, double lon);
    double getDistance(localCoord c);
    double getBearing(localCoord c);
    double getDistance();
    double getBearing();
    // double getBearing(double lat, double lon);
};

GNSSLocalization::GNSSLocalization(double lat, double lon) {
    baseLat = lat;
    baseLon = lon;
}

void GNSSLocalization::GNSSLocalizationProcess(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    double phim = (lat + baseLat) / 2 * M_PI / 180.0;
    localy = (lat - baseLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim));
    localx = (lon - baseLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim));
}

localCoord GNSSLocalization::getGNSSLocalization(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    localCoord coord;
    double phim = (lat + baseLat) / 2 * M_PI / 180.0;
    coord.y = (lat - baseLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim));
    coord.x = (lon - baseLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim));
    localx = coord.x;
    localy = coord.y;
    return coord;
}

double GNSSLocalization::getBearing(localCoord c) {
    return atan2(c.y, c.x) * 180.0 / M_PI;
}

double GNSSLocalization::getBearing() {
    return atan2(localy, localx) * 180.0 / M_PI;
}

double GNSSLocalization::getDistance() {
    return sqrt(pow(localx, 2) + pow(localy, 2));
}

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gnss.GNSSLocalizationProcess(msg->latitude, msg->longitude);
    ROS_INFO("x: %f, y: %f", gnss.localx, gnss.localy);
    ROS_INFO("distance: %f, bearing: %f", gnss.getDistance(), gnss.getBearing());
}

int main(int argc, char **argv) {
    gnss = GNSSLocalization(37.7749, -122.4194);
    ros::init(argc, argv, "GNSSLocalization");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ublox_gps", 1000, chatterCallback);
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("GPSOdometry", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        nav_msgs::Odometry msg;
        msg.pose.pose.position.x = gnss.localx;
        msg.pose.pose.position.y = gnss.localy;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}