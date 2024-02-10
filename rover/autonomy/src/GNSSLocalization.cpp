#include <iostream>
#include <cmath>
using namespace std;

typedef struct localCoord {
    double x;
    double y;
} localCoord;

class GNSSLocalization {
public:
    double baseLat;
    double baseLon;
    GNSSLocalization(double lat, double lon);
    localCoord getGNSSLocalization(double lat, double lon);
    double getDistance(localCoord c);
    double getBearing(localCoord c);
    // double getBearing(double lat, double lon);
};

GNSSLocalization::GNSSLocalization(double lat, double lon) {
    baseLat = lat;
    baseLon = lon;
}

localCoord GNSSLocalization::getGNSSLocalization(double lat, double lon) {
    localCoord coord;
    double phim = (lat + baseLat) / 2 * M_PI / 180.0;
    coord.y = (lat - baseLat) * (111.13209 - 0.56605 * cos(2 * phim) + 0.00120 * cos(4 * phim));
    coord.x = (lon - baseLon) * (111.41513 * cos(phim) - 0.09455 * cos(3 * phim) + 0.00012 * cos(5 * phim));
    return coord;
}

// double GNSSLocalization::getBearing(double lat, double lon) {
//     double y = sin((lon - baseLon) * M_PI / 180) * cos(lat * M_PI / 180);
//     double x = cos(baseLat * M_PI / 180) * sin(lat * M_PI / 180) - sin(baseLat * M_PI / 180) * cos(lat * M_PI / 180) * cos((lon - baseLon) * M_PI / 180);
//     double t = atan2(y, x);
//     double bearing = t * 180 / M_PI + 360;
//     return fmod(bearing, 360);
// }

double GNSSLocalization::getBearing(localCoord c) {
    return atan2(c.y, c.x) * 180.0 / M_PI;
}

double GNSSLocalization::getDistance(localCoord c) {
    return sqrt(pow(c.x, 2) + pow(c.y, 2));
}

int main() {
    GNSSLocalization gnss(43.645934,-79.441650);
    localCoord coord = gnss.getGNSSLocalization(43.651221,-79.430955);
    cout << "x: " << coord.x << ", y: " << coord.y << endl;
    cout << "distance: " << gnss.getDistance(coord) << endl;
    cout << "bearing xy: " << gnss.getBearing(coord) << endl;
    return 0;
}