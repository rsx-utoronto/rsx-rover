#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace std;

typedef struct localCoord {
    double x;
    double y;
} localCoord;

class GNSSLocalization {
private:
    double ENx;
    double ENy;
public:
    double baseLat;
    double baseLon;
    double frameBearing;
    double curLat;
    double curLon;
    double localx;
    double localy;
    GNSSLocalization(double lat, double lon, double bearing);
    GNSSLocalization();
    localCoord getGNSSLocalization(double lat, double lon);
    void GNSSLocalizationProcess(double lat, double lon);
    double getDistance(localCoord c);
    double getBearing(localCoord c);
    double getDistance();
    double getBearing();
    void setBase(double lat, double lon);
    void setBearing(double bearing);
    // double getBearing(double lat, double lon);
};

GNSSLocalization::GNSSLocalization(double lat, double lon, double bearing) {
    baseLat = lat;
    baseLon = lon;
    frameBearing = bearing * M_PI / 180.0;
}

GNSSLocalization::GNSSLocalization() {
    baseLat = 0;
    baseLon = 0;
}

void GNSSLocalization::GNSSLocalizationProcess(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    double phim = (lat + baseLat) / 2 * M_PI / 180.0;
    ENy = (lat - baseLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (lon - baseLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
    localy = ENy * cos(frameBearing) + ENx * sin(frameBearing);
    localx = ENx * cos(frameBearing) - ENy * sin(frameBearing);
}

localCoord GNSSLocalization::getGNSSLocalization(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    localCoord coord;
    double phim = (lat + baseLat) / 2 * M_PI / 180.0;
    ENy = (lat - baseLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (lon - baseLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
    coord.y = ENy * cos(frameBearing) + ENx * sin(frameBearing);
    coord.x = ENx * cos(frameBearing) - ENy * sin(frameBearing);
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

void GNSSLocalization::setBase(double lat, double lon) {
    baseLat = lat;
    baseLon = lon;
}

void GNSSLocalization::setBearing(double bearing) {
    frameBearing = bearing * M_PI / 180.0;
}

GNSSLocalization gnss = GNSSLocalization();

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gnss.GNSSLocalizationProcess(msg->latitude, msg->longitude);
    ROS_INFO("x: %f, y: %f", gnss.localx, gnss.localy);
    ROS_INFO("distance: %f, bearing: %f", gnss.getDistance(), gnss.getBearing());
}

int main(int argc, char **argv) {
    gnss.setBase(37.7749, -122.4194);
    gnss.setBearing(0);
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