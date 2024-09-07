#!/usr/bin/env python3

import rospy
import random
from rover.msg import StateMsg
from rover.srv import AddGoal
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

def publish_random_targets():
    rospy.init_node('random_target_publisher')
    pub = rospy.Publisher('target', Float64MultiArray, queue_size=10)
    #rate = rospy.Rate(1)  # Publish 1 random point per second

    while not rospy.is_shutdown():
        array_to_publish=[]
        for i in range(5):
            # Generate random (x, y) coordinates
            x = random.uniform(0, 10)
            y = random.uniform(0, 10)
            array_to_publish.extend([x,y])
        
        target_msg = Float64MultiArray() #wraps target message in data type
        target_msg.data = array_to_publish
    
        rospy.loginfo(f"Publishing random target: ({array_to_publish})")
        
        pub.publish(target_msg)

if __name__ == '__main__':
    try:
        publish_random_targets()
    except rospy.ROSInterruptException:
        pass        