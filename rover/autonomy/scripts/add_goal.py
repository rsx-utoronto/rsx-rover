#!/usr/bin/env python3


from math import *
import re
import rospy
#import tf
from nav_msgs.msg import Odometry


class GPS_to_UTM:

    # This class converts GPS coordinates to UTM metres
    # in the world frame

    ## CLASS VARIABLES ##
    # PARAMETERS
    RADIANS_PER_DEGREE = pi/180.0
    DEGREES_PER_RADIAN = 180.0/pi

    # Grid granularity for rounding UTM coordinates to generate MapXY.
    grid_size = 100000.0;    # 100 km grid

    # WGS84 Parameters
    WGS84_A =6378137.0   # major axis
    WGS84_B =6356752.31424518  # minor axis
    WGS84_F =0.0033528107    # ellipsoid flattening
    WGS84_E =0.0818191908    # first eccentricity
    WGS84_EP =0.0820944379    # second eccentricity

    # UTM Parameters
    UTM_K0  =  0.9996               # scale factor
    UTM_FE  = 500000.0             # false easting
    UTM_FN_N = 0.0                  # false northing, northern hemisphere
    UTM_FN_S = 10000000.0           # false northing, southern hemisphere
    UTM_E2   = (WGS84_E*WGS84_E)    # e^2
    UTM_E4   = (UTM_E2*UTM_E2)      # e^4
    UTM_E6   = (UTM_E4*UTM_E2)      # e^6
    UTM_EP2  = (UTM_E2/(1-UTM_E2))  # e'^2`

    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin (these is an example origin)

    def __init__(self, lat, lon, name, olon = -79.396042, olat= 43.660579):
        self.lat = lat
        self.lon = lon
        self.name = name
        self.olon = olon 
        self.olat = olat
        

    def get_xy_based_on_lat_long(self):
        # Define a local orgin, latitude and longitude in decimal degrees
        # GPS Origin
        
        xg2, yg2 = self.ll2xy()
        utmy, utmx, utmzone = self.LLtoUTM(self.lat, self.lon)

        # ROSPY Info Messages (wont print for some reason)
        rospy.loginfo("#########  "+self.name+"  ###########")  
        rospy.loginfo("LAT COORDINATES ==>"+str(self.lat)+","+str(self.lon))  
        rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
        rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

        # Normal Print Messages (but this will print for whatever reason)
        print("#########  "+self.name+"  ###########")  
        print("LAT COORDINATES ==>"+str(self.lat)+","+str(self.lon))  
        print("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
        print("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

        return xg2, yg2



    def ll2xy(self):
        '''
        Geonav: Lat/Long to X/Y
        Convert latitude and longitude in dec. degress to x and y in meters
        relative to the given origin location.  Converts lat/lon and orgin to UTM and then takes the difference

        Args:
        lat (float): Latitude of location
        lon (float): Longitude of location
        orglat (float): Latitude of origin location
        orglon (float): Longitude of origin location

        Returns:
        tuple: (x,y) where...
            x is Easting in m (local grid)
            y is Northing in m  (local grid)
        '''

        outmy, outmx, outmzone = self.LLtoUTM(self.olat, self.olon)
        utmy, utmx, utmzone = self.LLtoUTM(self.lat, self.lon)
        if (not (outmzone==utmzone)):
            print('WARNING: geonav_conversion: origin and location are in different UTM zones!')
        y = utmy-outmy
        x = utmx-outmx
        return (x,y) 


    def LLtoUTM(self, lat, lon):

        a = self.WGS84_A
        eccSquared = self.UTM_E2
        k0 = self.UTM_K0

        # Make sure the longitude is between -180.00 .. 179.9
        LongTemp = (lon+180.0)-int((lon+180.)/360.)*360.-180.
        LatRad = lat*self.RADIANS_PER_DEGREE
        LongRad = LongTemp*self.RADIANS_PER_DEGREE
        ZoneNumber = int((LongTemp + 180.0)/6.0) + 1

        if (lat >= 56.0 and self.lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
            ZoneNumber = 32
            # Special zones for Svalbard
        if (lat >= 72.0 and lat < 84.0 ):
            if ( LongTemp >= 0.0  and LongTemp <  9.0 ): ZoneNumber = 31
            elif ( LongTemp >= 9.0  and LongTemp < 21.0 ): ZoneNumber = 33
            elif ( LongTemp >= 21.0 and LongTemp < 33.0 ): ZoneNumber = 35
            elif ( LongTemp >= 33.0 and LongTemp < 42.0 ): ZoneNumber = 37
        # +3 puts origin in middle of zone
        LongOrigin = (ZoneNumber - 1.0)*6.0 - 180.0 + 3.0
        LongOriginRad = LongOrigin * self.RADIANS_PER_DEGREE

        # Compute the UTM Zone from the latitude and longitude
        UTMZone = "%d%s"%(ZoneNumber,self.UTMLetterDesignator())
        #print("UTM Zone: %s"%(UTMZone))
        eccPrimeSquared = (eccSquared)/(1.0-eccSquared)
        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad))
        T = tan(LatRad)*tan(LatRad)
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad)
        A = cos(LatRad)*(LongRad-LongOriginRad)
        
        M = a*((1 - eccSquared/4.0 - 3.0*eccSquared*eccSquared/64.0
                - 5.0*eccSquared*eccSquared*eccSquared/256.0) * LatRad
                - (3.0*eccSquared/8.0 + 3.0*eccSquared*eccSquared/32.0
                    + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(2.0*LatRad)
                + (15.0*eccSquared*eccSquared/256.0
                    + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(4.0*LatRad)
                - (35.0*eccSquared*eccSquared*eccSquared/3072.0)*sin(6.0*LatRad))

        UTMEasting = (k0*N*(A+(1.0-T+C)*A*A*A/6.0
                            + (5.0-18.0*T+T*T+72*C
                                - 58.0*eccPrimeSquared)*A*A*A*A*A/120.0)
                        + 500000.0)

        UTMNorthing = (k0*(M+N*tan(LatRad)
                            *(A*A/2.0+(5.0-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
                            + (61.0-58.0*T+T*T+600.0*C
                                - 330.0*eccPrimeSquared)*A*A*A*A*A*A/720.0)));
        if (lat < 0):
            # 10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0
        
        return (UTMNorthing, UTMEasting, UTMZone)

    def UTMLetterDesignator(self):
        
        LetterDesignator =""

        if ((84 >= self.lat) and (self.lat >= 72)):  LetterDesignator = 'X'
        
        elif ((72 > self.lat) and (self.lat >= 64)):  LetterDesignator = 'W'
        elif ((64 > self.lat) and (self.lat >= 56)):  LetterDesignator = 'V'
        elif ((56 > self.lat) and (self.lat >= 48)):  LetterDesignator = 'U'
        elif ((48 > self.lat) and (self.lat >= 40)):  LetterDesignator = 'T'
        elif ((40 > self.lat) and (self.lat >= 32)):  LetterDesignator = 'S'
        elif ((32 > self.lat) and (self.lat >= 24)):  LetterDesignator = 'R'
        elif ((24 > self.lat) and (self.lat >= 16)):  LetterDesignator = 'Q'
        elif ((16 > self.lat) and (self.lat >= 8)) :  LetterDesignator = 'P'
        elif (( 8 > self.lat) and (self.lat >= 0)) :  LetterDesignator = 'N'
        elif (( 0 > self.lat) and (self.lat >= -8)):  LetterDesignator = 'M'
        elif ((-8 > self.lat) and (self.lat >= -16)): LetterDesignator = 'L'
        elif ((-16 > self.lat) and (self.lat >= -24)): LetterDesignator = 'K'
        elif ((-24 > self.lat) and (self.lat>= -32)): LetterDesignator = 'J'
        elif ((-32 > self.lat) and (self.lat >= -40)): LetterDesignator = 'H'
        elif ((-40 > self.lat) and (self.lat >= -48)): LetterDesignator = 'G'
        elif ((-48 > self.lat) and (self.lat >= -56)): LetterDesignator = 'F'
        elif ((-56 > self.lat) and (self.lat >= -64)): LetterDesignator = 'E'
        elif ((-64 > self.lat) and (self.lat >= -72)): LetterDesignator = 'D'
        elif ((-72 > self.lat) and (self.lat >= -80)): LetterDesignator = 'C'
            # 'Z' is an error flag, the Latitude is outside the UTM limits
        else: LetterDesignator = 'Z'
        return LetterDesignator


    def convertGPSToOdom(self):
        
        pass 


def convertSignToOdom():

    # convert distance and heading to UTM coords 

    # convert UTM to odom (transform provided by robot localization node)
    pass

if __name__ == '__main__':
    
    coords1 = GPS_to_UTM(lat=40.9,lon=7.9, name="MAP") #EDIT (these are example lat, long coords)
    coords1.get_xy_based_on_lat_long()


