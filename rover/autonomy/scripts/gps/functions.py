#!/usr/bin/python3
"""
This script contains the following functions:
    getHeadingBetweenGPSCoordinates()
    getDistanceBetweenGPSCoordinates()
    getAddedAngles()
    eulerToQuaternion()
"""
from math import cos, sin, atan2, pi, degrees, radians
from geopy import distance
from typing import Tuple


def getHeadingBetweenGPSCoordinates(lat1: float, long1: float, lat2: float, long2: float) -> float:
    """
    This function takes as input the latitude and longitude of two coordinates (1 and 2) and returns
    the heading (in radians range -pi to pi, with North as 0.0) from the first coordinate to the second. Assumes lat and
    long values are in decimal degrees.
    """
    # converting our degree inputs to radians for calculation
    la1 = radians(lat1)
    la2 = radians(lat2)
    lo1 = radians(long1)
    lo2 = radians(long2)
    # calculate heading
    deltaLo = lo2 - lo1
    y = cos(la2)*sin(deltaLo)
    x = cos(la1)*sin(la2) - sin(la1)*cos(la2)*cos(deltaLo)
    heading = atan2(y,x)
    return heading


def getDistanceBetweenGPSCoordinates(latLong1: Tuple[float, float], latLong2: Tuple[float, float]) -> float:
    """
    This function takes as input the latitude and longitude of two coordinates and returns
    the absolute distance (in meters) from the first coordinate to the second. Assumes lat and
    long values are in decimal degrees.
    """
    # we use the geopy library to get the distance in km
    dist = distance.distance(latLong1, latLong2).km
    # we convert to meters
    dist = dist*1000
    return dist


def getAddedAngles(theta, phi):
    """
    Given 2 angles (in range -pi to pi) we add them and return the new angle in range (-pi, pi].
    """
    # first add then add pi since they were originally in the range (-pi,pi] and we need (o, 2pi] 
    comb = theta + phi + pi
    # next we get the result modulo 2pi and subtract pi to get the angle in the (-pi, pi] range
    comb = (comb % (2*pi)) - pi
    # note that because of how modulo works angles that should be pi will be -pi so we make a catch for that
    if comb == -pi:
        comb = pi
    return comb


def eulerToQuaternion(roll: float, pitch: float, yaw: float) -> Tuple[float,float,float,float]:
    """
    Given the roll, pitch, and yaw angles, we calculate and return the quaternion for it 
    """
    # we first calculate the parts for the conversion to quaternion to make it easier to read
    sinRoll = sin(roll/2)
    cosRoll = cos(roll/2)
    sinPitch = sin(pitch/2)
    cosPitch = cos(pitch/2)
    sinYaw = sin(yaw/2)
    cosYaw = cos(yaw/2)
    # now we calculate the parts of the quaternion
    qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
    qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
    qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw
    qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
    return (qx,qy,qz,qw)
