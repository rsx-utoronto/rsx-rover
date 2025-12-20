#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

class RTABMapInitialPose(Node):
    def __init__(self):
        # Initialize the ROS node
        super().__init__('rtabmap_initialpose_node')
        
        # rospy.Subscriber("/pose", PoseStamped, self.pose_callback)
        self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        # self.initialpose_service = rospy.ServiceProxy('rtabmap/initialpose', Empty)
        self.initialpose_service = self.create_client(Empty, 'rtabmap/initialpose')
        # self.cli = self.create_client(Empty, 'rtabmap/initialpose')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        
        
        # self.pub = rospy.Publisher('rtabmap/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'rtabmap/initialpose', 10)
        self.initialPose = PoseWithCovarianceStamped()
        
    def pose_callback(self, data):
        self.initialPose.header = data.header
        self.initialPose.pose.pose = data.pose
        self.initialPose.pose.pose.position.z = 0.0
        self.initialPose.pose.covariance = [0] * 36  # Initialize covariance to zero

    def call_rtabmap_initialpose(self):
        
        # Wait for the service to become available
        # rospy.loginfo("Waiting for rtabmap/initialpose service...")
        self.get_logger().info("Waiting for rtabmap/initialpose service...")
        # rospy.wait_for_service('rtabmap/initialpose')
        while not self.initialpose_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        try:
            # Publish the pose (RTABMap typically listens for published poses)
            while self.initialPose.header.seq == 0:
                self.get_logger().info("Waiting for pose data...")
                rclpy.sleep(1)
                
            self.pub.publish(self.initialPose)
            
            # Call the service (if it takes an Empty request)
            response = self.initialpose_service()
            
            self.get_logger().info("Initial pose set successfully")
            return response
            
        except rospy.ServiceException as e:
            
            self.get_logger().error(f"Service call failed: {e}")

if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        rtabmap_initialpose = RTABMapInitialPose()
        rtabmap_initialpose.call_rtabmap_initialpose()
        rclpy.spin(rtabmap_initialpose)
    except rclpy.exceptions.ROSInterruptException:
        pass
