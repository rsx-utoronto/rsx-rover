#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def generate_fixed_gps_data():
    """Generate fixed GPS data for testing."""
    # Set fixed latitude and longitude values
    latitude = 38.4063  # Example: San Francisco, CA
    longitude = -110.7918  # Example: San Francisco, CA

    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.header.frame_id = "gps_frame"
    gps_msg.latitude = latitude
    gps_msg.longitude = longitude
    gps_msg.altitude = 0.0  # Set altitude to 0 for simplicity
    gps_msg.status.status = gps_msg.status.STATUS_FIX  # Assume a good GPS fix
    gps_msg.status.service = gps_msg.status.SERVICE_GPS  # GPS service
    return gps_msg

def gps_publisher():
    """Publish fixed GPS data to the 'gps' topic."""
    rospy.init_node('gps_publisher', anonymous=True)
    pub = rospy.Publisher('gps', NavSatFix, queue_size=10)

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        gps_data = generate_fixed_gps_data()
        pub.publish(gps_data)
        rospy.loginfo(f"Published GPS data: Latitude={gps_data.latitude}, Longitude={gps_data.longitude}")
        rate.sleep()

if __name__ == "__main__":
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
