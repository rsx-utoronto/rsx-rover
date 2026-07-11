#!/usr/bin/python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from inertial_sense_ros.msg import GPS
from add_goal import *
from tf.transformations import *

class GPSTesting: 

    def __init__(self):

        self.gps_sub = rospy.Subscriber("/gps", GPS, self.gps_callback)
        self.ins_sub = rospy.Subscriber("/ins", Odometry, self.ins_callback)
        self.utm_pub = rospy.Publisher("/utm", Odometry, queue_size=1)
        self.ins_pub = rospy.Publisher("/ins_transformed", Odometry, queue_size=1)
        self.first_gps = True
        self.first_ins = True
        self.olon = -1 
        self.olat = -1
        self.oins_x = -1
        self.oins_y = -1
    
    def gps_callback(self, msg):

        # read in gps 
        self.gps_msg = msg
        print(self.gps_msg)

        self.lat = self.gps_msg.latitude
        self.long = self.gps_msg.longitude

        if self.first_gps == True:
            self.olon = self.long
            self.olat = self.lat
            self.first_gps = False

        # run the utm conversion 
        utm_msg = Odometry()

        gps_to_utm = GPS_to_UTM(self.lat, self.long, "test")
        gps_to_utm.olat = self.olat
        gps_to_utm.olon = self.olon
        # self.x, self.y = gps_to_utm.get_xy_based_on_lat_long()
        self.x, self.y, _ = gps_to_utm.convertGPSToOdom()
        utm_msg.header.frame_id = "ins"
        utm_msg.child_frame_id = ""
        utm_msg.pose.pose.position.x = self.x 
        utm_msg.pose.pose.position.y = self.y

        # Publish to topic
        self.utm_pub.publish(utm_msg)
    
    def ins_callback(self, msg):

        ins_msg = msg

        if self.first_ins:
            self.oins_msg = ins_msg
            self.first_ins = False
        
        ins_new_msg = Odometry()
        ins_new_msg.header.frame_id = "ins"
        ins_new_msg.child_frame_id = ""
        ins_new_msg.pose.pose.position.x = ins_msg.pose.pose.position.x - self.oins_msg.pose.pose.position.x
        ins_new_msg.pose.pose.position.y = ins_msg.pose.pose.position.y - self.oins_msg.pose.pose.position.y
        ins_new_msg.pose.pose.position.z = ins_msg.pose.pose.position.z - self.oins_msg.pose.pose.position.z
        
        q1_inv = quaternion_from_euler(0, 0, 0)
        q2 = quaternion_from_euler(0, 0, 0)
        q1_inv[0] = self.oins_msg.pose.pose.orientation.x
        q1_inv[1] = self.oins_msg.pose.pose.orientation.y
        q1_inv[2] = self.oins_msg.pose.pose.orientation.z
        q1_inv[3] = -self.oins_msg.pose.pose.orientation.w # Negate for inverse

        q2[0] = ins_msg.pose.pose.orientation.x
        q2[1] = ins_msg.pose.pose.orientation.y
        q2[2] = ins_msg.pose.pose.orientation.z
        q2[3] = ins_msg.pose.pose.orientation.w

        qr = quaternion_multiply(q2, q1_inv)

        ins_new_msg.pose.pose.orientation.x = qr[0]
        ins_new_msg.pose.pose.orientation.y = qr[1]
        ins_new_msg.pose.pose.orientation.z = qr[2]
        ins_new_msg.pose.pose.orientation.w = qr[3]

        # Publish to topic
        self.ins_pub.publish(ins_new_msg)


def main():
    rospy.init_node("converter_node")

    gps_test = GPSTesting()

    rospy.spin()


if __name__ == "__main__":
    main()