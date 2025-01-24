#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from math import atan2, pi, sin, cos
import gps_conversion_functions as functions
import message_filters
from calian_gnss_ros2_msg.msg import GnssSignalStatus


# important constants, make sure these are accurate to the current state of the rover (read below description)
# doens't matter which antenna is 1 or 2, the positive y axis is the direction of the front of the rover and
# the positive x axis is to the right, give the units don't matter as long as you are consitent for all
X_POS_ANTENNA_1 = -1.0
Y_POS_ANTENNA_1 = 0.0
X_POS_ANTENNA_2 = 0.0
Y_POS_ANTENNA_2 = 0.0
# the NavSatFix publishers should correspond to the same numbered antenna from the position above
GPS_ANTENNA_1 = "calian_gnss/gps_extended"
GPS_ANTENNA_2 = "calian_gnss/base_gps_extended"


# our heading is calculated as the direction from antenna 1 to 2, so we 
# need to find the correction angle that places the heading as the direction the rover is moving by
# using the positions of the antennas
# we first get the heading from antenna 1 to 2, note atan2 returns the direction from 1 to 2 in (-pi, pi]
# scale so if antenna 1 is in the middle then 2 being directly up returns pi/2 or directly right returns 0
# for example
ORIG_HEADING = atan2(Y_POS_ANTENNA_2-Y_POS_ANTENNA_1, X_POS_ANTENNA_2-X_POS_ANTENNA_1)
# we know (since we mandated it above) that the front of the rover would be in direction x=0,y=1 which
# using the same scale as above puts the heading at pi/2, if the calculated heading between antenna is not
# this then we need to correct any given angle by the difference (note pi is added first to simiplify there
# being positive and negative values for ORIG_HEADING)
ANGLE_CORRECTION = pi/2 - ORIG_HEADING
# the above angle correct means if we calculate the heading from antenna 1 to 2, then add the correction
# we get the heading of the direction of the front of the rover


class GPSToPose: 
    
    def __init__(self, origin_coordinates=None):
        rospy.init_node("gps_to_pose")
        self.origin_coordinates = origin_coordinates
        self.gps1 = message_filters.Subscriber(GPS_ANTENNA_1, GnssSignalStatus)
        self.gps2 = message_filters.Subscriber(GPS_ANTENNA_2, GnssSignalStatus)
        # here 5 is the size of the queue and 0.2 is the time allowed between messages
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps1, self.gps2], 5, 0.2)
        self.ts.registerCallback(self.callback)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
        self.msg = PoseStamped()
        self.x = 0
        self.y = 0


    def callback(self, gps1, gps2):
        # first we get the most recent longitudes and latitudes
        lat1 = gps1.latitude
        long1 = gps1.longitude
        lat2 = gps2.latitude
        long2 = gps2.longitude

        # let's first calculate our heading
        heading = functions.getHeadingBetweenGPSCoordinates(lat1, long1, lat2, long2)
        # now we apply the angle correction so its the heading towards the front of the rover
        heading = functions.getAddedAngles(heading, ANGLE_CORRECTION)
        # now we convert our angle to a quaternion for our pose data
        qx,qy,qz,qw = functions.eulerToQuaternion(0.0, 0.0, heading)

        # now let's get our coordinate
        # if the value for origin_coordinates is None then this is the first callback and we set the current
        # current gps location as the origin
        # note that we consider north (or 0.0 as a calculated gps heading on the -pi to pi scale) to be the
        # positive y direction for our coordinate system 
        if self.origin_coordinates is None:
            # note that since the gps antenna locations are fixed distances from the origin, to not have to
            # we use the coordinates for antenna one, then apply a fix based of where antenna 1 is from the
            # center of the rover if needed afterwards
            self.origin_coordinates = (lat1, long1)
            x = 0
            y = 0
        else:
            # it is difficult to calculate from decimal degrees the x and y distance between two points
            # this is because a degree of longitude at te equator is a different distance that at 23 degree
            # North, for example
            # thus, I first get the distance and angle between the origin and our current rover position,
            # then use these value to convert to our coordinate system (where North is 0.0 degrees)
            orig_lat, orig_long = self.origin_coordinates
            distance = functions.getDistanceBetweenGPSCoordinates((orig_lat, orig_long), (lat1, long1))
            theta = functions.getHeadingBetweenGPSCoordinates(orig_lat, orig_long, lat1, long1)
            # since we measure from north y is r*cos(theta) and x is -r*sin(theta)
            self.x = -distance * sin(theta)
            self.y = distance * cos(theta)

        # now that we have the coordinate and angle quaternion information we can return the pose message
        msg = self.msg
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)


def main():
    gps_converter = GPSToPose(None)
    rospy.spin()


if __name__ == "__main__":
    main()