#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class DumbMoveitInterface():

    def __init__(self):
        rospy.init_node("dumb_moveit_interface")
        rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.moveitAngles)
        rospy.Subscriber("/arm_state", String, self.armState)

        self.goalPub = rospy.Publisher("/arm_goal_pos", Float32MultiArray, queue_size=10)
        self.trajectoryQueue = []
        self.rate = rospy.Rate(1)
        self.curState = "Idle"

    def moveitAngles(self, actionGoal: ExecuteTrajectoryActionGoal):
        points = actionGoal.goal.trajectory.joint_trajectory.points
        self.trajectoryQueue.append(points)
        # rospy.logdebug("recv'd")

    def armState(self, state: String):
        self.curState = state.data
    
    def main(self):
        while not rospy.is_shutdown():
            if(len(self.trajectoryQueue) > 0):
                for i, points in enumerate(self.trajectoryQueue[0][1:]):
                    goalData = Float32MultiArray()
                    # goalData.data = np.rad2deg(points.position)

                    prev = np.array(self.trajectoryQueue[0][i].positions)
                    delta = np.array(points.positions) - prev 
                    increment = delta/10

                    for j in range(10): 
                        goalData.data = prev + (j+1)*increment
                        self.publishGoal(goalData)
                        self.rate.sleep()
                    # rospy.logdebug(points.positions)
                    # self.rate.sleep()
                self.trajectoryQueue.pop(0)
    
    def publishGoal(self, goalData):
        if self.curState == "IK":
            self.goalPub.publish(goalData)

if __name__ == "__main__":
    node = DumbMoveitInterface()
    node.main() 
    # rospy.spin()
