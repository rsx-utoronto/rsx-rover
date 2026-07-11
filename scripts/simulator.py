import rospy
import math 
import time
from typing import Tuple
from geometry_msgs.msg import Twist, PoseStamped

class Simulator():
    def __init__(self, x, y, heading, time_step):
        self.drive_cmd = rospy.Subscriber("/drive", Twist, self.drive_callback)
        self.write_pose = rospy.Publisher("pose", PoseStamped, queue_size=10)
        self.vel = Twist()
        self.sim_pose = PoseStamped()
        self.quat = Quaternion()
        self.sim_pose.header.seq = 0
        self.sim_pose.header.stamp.secs = 0
        self.sim_pose.header.stamp.nsecs = 0
        self.sim_pose.header.frame_id = "map"
        self.sim_pose.pose.position.x = x
        self.sim_pose.pose.position.y = y   
        self.sim_pose.pose.position.z = 0
        self.quat.set_heading_to_quaternion(heading)
        self.sim_pose.pose.orientation.x = self.quat.x
        self.sim_pose.pose.orientation.y = self.quat.y
        self.sim_pose.pose.orientation.z = self.quat.z
        self.sim_pose.pose.orientation.w = self.quat.w
        self.time_step = time_step
    def drive_callback(self, data):
        self.vel = data
    def publish_pose(self):
        heading = self.quat.quaternion_to_heading(self.sim_pose.pose.orientation)
        heading += self.vel.angular.z * self.time_step / 2
        self.sim_pose.pose.position.x += self.vel.linear.x * self.time_step * math.cos(heading)
        self.sim_pose.pose.position.y += self.vel.linear.x * self.time_step * math.sin(heading)
        heading += self.vel.angular.z * self.time_step / 2
        self.quat.set_heading_to_quaternion(heading)
        self.sim_pose.pose.orientation.x = self.quat.x
        self.sim_pose.pose.orientation.y = self.quat.y
        self.sim_pose.pose.orientation.z = self.quat.z
        self.sim_pose.pose.orientation.w = self.quat.w
        self.write_pose.publish(self.sim_pose)


class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def set_heading_to_quaternion(self, heading):
        # Convert heading (in degrees) to radians
        heading_rad = math.radians(heading)
        
        # Calculate quaternion components
        self.w = math.cos(heading_rad / 2)
        self.x = 0
        self.y = 0
        self.z = math.sin(heading_rad / 2)
        
        return self
    
    def quaternion_to_heading(self, data):
        # Convert input quaternion to heading (in radians)
        return math.atan2(2*(data.w*data.z + data.x*data.y), 1 - 2*(data.y**2 + data.z**2))
    
    def set(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        return self

if __name__ == '__main__':
    rospy.init_node('2d_sim')
    simulator = Simulator(0, 0, 0, 0.01)

    while not rospy.is_shutdown():
        simulator.publish_pose()
        time.sleep(0.01)

