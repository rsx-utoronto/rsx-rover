#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray

class DumbMoveitInterface():

    def __init__(self):
        rospy.init_node("dumb_moveit_interface")
        rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.moveitAngles)

        self.goalPub = rospy.Publisher("/arm_goal_pos", Float32MultiArray, queue_size=10)
        self.trajectoryQueue = []
        self.rate = rospy.Rate(1)


    def moveitAngles(self, actionGoal: ExecuteTrajectoryActionGoal):
        points = actionGoal.goal.trajectory.joint_trajectory.points
        self.trajectoryQueue.append(points)
        # rospy.logdebug("recv'd")
    
    def main(self):
        while not rospy.is_shutdown():
            if(len(self.trajectoryQueue) > 0):
                for points in self.trajectoryQueue[0]:
                    goalData = Float32MultiArray()
                    goalData.data = points.positions
                    self.goalPub.publish(goalData)
                    # rospy.logdebug(points.positions)
                    self.rate.sleep()
                self.trajectoryQueue.pop(0)

if __name__ == "__main__":
    node = DumbMoveitInterface()
    node.main() 
    # rospy.spin()
