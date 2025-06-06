#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// #include "rover/StateMsg.h"

using namespace std;

typedef struct localCoord {
    double x;
    double y;
} localCoord;

class GNSSTargetAid {
private:
    double ENx;
    double ENy;
public:
    double targetLat;
    double targetLon;
    double frameBearing;
    double curLat;
    double curLon;
    double localx;
    double localy;
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
}

void GNSSTargetAid::GNSSLocalizationProcess(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    double phim = (lat + targetLat) / 2 * M_PI / 180.0;
    ENy = (lat - targetLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (lon - targetLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
    localy = ENy * cos(frameBearing) + ENx * sin(frameBearing);
    localx = ENx * cos(frameBearing) - ENy * sin(frameBearing);
}

localCoord GNSSTargetAid::getGNSSLocalization(double lat, double lon) {
    curLat = lat;
    curLon = lon;
    localCoord coord;
    double phim = (lat + targetLat) / 2 * M_PI / 180.0;
    ENy = (lat - targetLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim)) * 1000;
    ENx = (lon - targetLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim)) * 1000;
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

GNSSTargetAid gnss = GNSSTargetAid();

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gnss.GNSSLocalizationProcess(msg->latitude, msg->longitude);
    ROS_INFO("x: %f, y: %f", gnss.localx, gnss.localy);
    ROS_INFO("distance: %f, bearing: %f", gnss.getDistance(), gnss.getBearing());
}

int main(int argc, char **argv) {
    const double THRESHOLD = 3;

    gnss.setTarget(43.139199, -79.114206);
    gnss.setBearing(0);
    ros::init(argc, argv, "GNSSLocalization");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ublox/fix", 1000, chatterCallback);
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("GPSOdometry", 1000);

    // State machine publisher
    rover::StateMsg stateMsg;
    ros::Publisher statePublisher = n.advertise<rover::StateMsg>("/gps_checker_node/rover_state", 10);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        nav_msgs::Odometry msg;
        msg.pose.pose.position.x = gnss.localx;
        msg.pose.pose.position.y = gnss.localy;

        // State machine message
        stateMsg.GPS_GOAL_REACHED = (gnss.getDistance() < THRESHOLD) ?true :false;
        statePublisher.publish(stateMsg);

        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
