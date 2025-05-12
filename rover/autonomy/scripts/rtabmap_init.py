#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

class RTABMapInitialPose:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rtabmap_initialpose_client', anonymous=True)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)
        self.initialpose_service = rospy.ServiceProxy('rtabmap/initialpose', Empty)
        self.pub = rospy.Publisher('rtabmap/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.initialPose = PoseWithCovarianceStamped()
        
    def pose_callback(self, data):
        self.initialPose.header = data.header
        self.initialPose.pose.pose = data.pose
        self.initialPose.pose.covariance = [0] * 36  # Initialize covariance to zero

    def call_rtabmap_initialpose(self):
        
        # Wait for the service to become available
        rospy.loginfo("Waiting for rtabmap/initialpose service...")
        rospy.wait_for_service('rtabmap/initialpose')
        
        try:
            # Publish the pose (RTABMap typically listens for published poses)
            while self.initialPose.header.seq == 0:
                rospy.loginfo("Waiting for pose data...")
                rospy.sleep(1)
            self.pub.publish(self.initialPose)
            
            # Call the service (if it takes an Empty request)
            response = self.initialpose_service()
            rospy.loginfo("Initial pose set successfully")
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        rtabmap_initialpose = RTABMapInitialPose()
        rtabmap_initialpose.call_rtabmap_initialpose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
