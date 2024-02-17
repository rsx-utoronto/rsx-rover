#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
import math
import tf.transformations as tfs

def publish_transform_static(heading: int):
    rospy.init_node("utm_to_map")
    
    utm_to_map = StaticTransformBroadcaster()
    static_tf = TransformStamped()


    static_tf.header.frame_id = "utm"
    static_tf.child_frame_id = "map"


    utm_static = rospy.wait_for_message("/utm", Odometry)

    # Translation
    static_tf.transform.translation.x = utm_static.pose.pose.position.x
    static_tf.transform.translation.y = utm_static.pose.pose.position.y
    static_tf.transform.translation.z = utm_static.pose.pose.position.z

    # Rotation
    heading = math.radians(heading)

    quat = tfs.quaternion_from_euler(0, 0, heading)

    static_tf.transform.rotation.x = quat[0]
    static_tf.transform.rotation.y = quat[1]
    static_tf.transform.rotation.z = quat[2]
    static_tf.transform.rotation.w = quat[3]

    utm_to_map.sendTransform(static_tf)

    rospy.spin()
    
    

if __name__ == "__main__":
    try:
        heading = int(input("Heading (positive integer between 0 and 359, clockwise is increasing): "))
        print(f"Using : {heading}")
    except ValueError:
        print("Integer Please...")

    publish_transform_static(heading)
