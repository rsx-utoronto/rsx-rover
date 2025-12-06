#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
import random

class TestPublishers(Node):
    def __init__(self):
        super().__init__('test_publishers')

        # Publishers
        self.aruco_found_pub = self.create_publisher(Bool, 'aruco_found', 10)
        self.aruco_name_pub = self.create_publisher(String, 'aruco_name', 10)
        self.led_colour_pub = self.create_publisher(String, '/led_colour', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/calian_gnss/gps', 10)
        self.status_pub = self.create_publisher(String, '/status', 10)

        # Initial state
        self.aruco_names = ["Aruco_1", "Aruco_2", "Aruco_3"]
        self.led_colors = ["red", "green", "yellow"]
        self.statuses = ["Idle", "Processing", "Complete", "Error"]
        self.latitude = 38.4
        self.longitude = -110.78
        self.aruco_index = 0
        self.led_index = 0

        # Timers
        self.create_timer(2.0, self.publish_aruco_found)
        self.create_timer(1.0, self.publish_aruco_name)
        self.create_timer(3.0, self.publish_led_colour)
        self.create_timer(1.0, self.publish_gps_coordinates)
        self.create_timer(5.0, self.publish_status)

        # Toggle state
        self.aruco_found_state = True

    def publish_aruco_found(self):
        msg = Bool()
        msg.data = self.aruco_found_state
        self.aruco_found_pub.publish(msg)
        self.get_logger().info(f"Published: Aruco Found = {msg.data}")
        self.aruco_found_state = not self.aruco_found_state

    def publish_aruco_name(self):
        name = self.aruco_names[self.aruco_index]
        msg = String()
        msg.data = name
        self.aruco_name_pub.publish(msg)
        self.get_logger().info(f"Published: Aruco Name = {name}")
        self.aruco_index = (self.aruco_index + 1) % len(self.aruco_names)

    def publish_led_colour(self):
        color = self.led_colors[self.led_index]
        msg = String()
        msg.data = color
        self.led_colour_pub.publish(msg)
        self.get_logger().info(f"Published: LED Colour = {color}")
        self.led_index = (self.led_index + 1) % len(self.led_colors)

    def publish_gps_coordinates(self):
        self.latitude += random.uniform(-0.0001, 0.0001)
        self.longitude += random.uniform(-0.0001, 0.0001)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = random.uniform(0, 500)

        self.gps_pub.publish(msg)
        self.get_logger().info(f"Publishing GPS coordinates: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}")

    def publish_status(self):
        status = random.choice(self.statuses)
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Published: Status = {status}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublishers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



####################################
#ROS 1: 
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Bool, String
# import time
# import random
# import threading

# def aruco_found_publisher():
#     aruco_found_pub = rospy.Publisher('aruco_found', Bool, queue_size=10)
#     while not rospy.is_shutdown():
#         aruco_found_pub.publish(True)
#         rospy.loginfo("Published: Aruco Found = True")
#         time.sleep(2)
#         aruco_found_pub.publish(False)
#         rospy.loginfo("Published: Aruco Found = False")
#         time.sleep(2)

# def aruco_name_publisher():
#     aruco_name_pub = rospy.Publisher('aruco_name', String, queue_size=10)
#     aruco_names = ["Aruco_1", "Aruco_2", "Aruco_3"]
#     while not rospy.is_shutdown():
#         for name in aruco_names:
#             aruco_name_pub.publish(name)
#             rospy.loginfo(f"Published: Aruco Name = {name}")
#             time.sleep(1)

# def led_colour_publisher():
#     led_colour_pub = rospy.Publisher('/led_colour', String, queue_size=10)
#     led_colors = ["red", "green", "yellow"]
#     while not rospy.is_shutdown():
#         for color in led_colors:
#             led_colour_pub.publish(color)
#             rospy.loginfo(f"Published: LED Colour = {color}")
#             time.sleep(3)

# def publish_gps_coordinates():
#     gps_pub = rospy.Publisher('/calian_gnss/gps', NavSatFix, queue_size=10)
#     rate = rospy.Rate(1)
#     latitude = 38.4
#     longitude = -110.78
#     while not rospy.is_shutdown():
#         gps_msg = NavSatFix()
#         gps_msg.header.stamp = rospy.Time.now()
#         gps_msg.header.frame_id = "gps"
#         latitude += random.uniform(-0.0001, 0.0001)
#         longitude += random.uniform(-0.0001, 0.0001)
#         gps_msg.latitude = latitude
#         gps_msg.longitude = longitude
#         gps_msg.altitude = random.uniform(0, 500)
#         rospy.loginfo(f"Publishing GPS coordinates: Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}, Altitude: {gps_msg.altitude}")
#         gps_pub.publish(gps_msg)
#         rate.sleep()

# def status_publisher():
#     status_pub = rospy.Publisher('/status', String, queue_size=10)
#     statuses = ["Idle", "Processing", "Complete", "Error"]
#     while not rospy.is_shutdown():
#         status = random.choice(statuses)
#         status_pub.publish(status)
#         rospy.loginfo(f"Published: Status = {status}")
#         time.sleep(5)

# if __name__ == '__main__':
    
#     rospy.init_node('test_publishers')
#     threading.Thread(target=aruco_found_publisher, daemon=True).start()
#     threading.Thread(target=aruco_name_publisher, daemon=True).start()
#     threading.Thread(target=led_colour_publisher, daemon=True).start()
#     threading.Thread(target=publish_gps_coordinates, daemon=True).start()
#     threading.Thread(target=status_publisher, daemon=True).start()
#     rospy.spin()
