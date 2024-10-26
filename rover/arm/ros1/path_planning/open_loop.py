import numpy as np
import APF
import APF_func as af
from std_msgs.msg import Float32MultiArray
import rospy

#initialize the variables that are used to store the starting position, goal position, and obstacle locations
start = np.empty((1,3))
goal = np.empty((0,3))
obstacles = np.empty((0,3))

def start_callback(msg):
    #collect start position data from its corresponding topic
    if first_run == True:
        start = np.array(msg)
        first_run = False

def goal_callback(msg):
    #collect goal position data from its corresponding topic
    global goal
    goal = np.array(msg.data).reshape(1,3)

def obstacles_callback(msg):
    #collect obstacle data from its corresponding topic
    global obstacles
    new_row = np.array([msg.data])
    obstacles = np.vstack([obstacles, new_row])



def move():
    global start
    global goal
    global obstacles

    rospy.Subscriber("/ARM_CUR_POS", Float32MultiArray, start_callback, queue_size=10) #topic doesn't exist yet
    rospy.Subscriber("/ARM_GOAL_POS", Float32MultiArray, goal_callback, queue_size=10)

    #/ARM_CUR_POS

    min_dist = 0.5 #minimum distance between the tool tip and the goal to stop the arm
    apf = APF(start, goal, obstacles)

    while np.linalg.norm(goal - start) <= min_dist:

        apf.move()
