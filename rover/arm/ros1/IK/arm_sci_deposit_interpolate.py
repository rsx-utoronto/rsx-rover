import rospy
from std_msgs.msg import Float32MultiArray
from numpy import deg2rad, rad2deg
# from arm_sci import SciArm, ArmSciNode

class DepositInterpolateNode:
    def __init__(self):
        # self.curr_pos = Float32MultiArray()
        self.curr_pos = [0, 0, 0, 0, 0]
        rospy.init_node("deposit_interpolate")
        rospy.Subscriber("deposit_target", Float32MultiArray, self.goToDeposit)
        self.targetAnglePub = rospy.Publisher("PATH_PLANNING", Float32MultiArray, queue_size=10)

        rospy.Subscriber("arm_curr_pos", Float32MultiArray, self.onCurrPosUpdate)
        rospy.Subscriber("arm_goal_pos", Float32MultiArray, self.onCurrPosUpdate) # Leave for now, since curr_pos isn't implemented in rviz

        # self.dhTable = [[79.7, 0, 0, 0],
        #            [0,    0, 367, 0],
        #            [0,    0, 195, 0],
        #            [0,    0, 67, 0],
        #            [92,   0, 0, 0]]

        self.rate = rospy.Rate(30)

    # def rotateBaseToAngle(self, depositTheta):
    #     pass
    
    def interpolateAngles(self, endTarget):
        pass

    def onCurrPosUpdate(self, data):
        self.curr_pos = (data.data)
        print(f"curr_pos: {self.curr_pos}")

    def goToDeposit(self, depositTarget): # All pos coords are in cylindrical (theta, r, z, alpha)
        # self.arm.cylTarget = depositTarget
        # print(self.curAngleQueue.qsize())

        goalAngles = Float32MultiArray()
        goalAngles.data.append(depositTarget.data[0])
        goalAngles.data.extend(self.curr_pos[1:]) # Copy the other angles

        self.targetAnglePub.publish(goalAngles) # First move base (will be put in a buffer)

        print(f"goalAngles: {goalAngles.data}")

        self.targetAnglePub.publish(depositTarget) # Next move arm to deposit (next in buffer)

        # self.interpolateAngles(depositTarget)
    
    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    node = DepositInterpolateNode()
    node.main()