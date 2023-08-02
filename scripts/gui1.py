#!/usr/bin/env python3
import rospy
import subprocess
from geometry_msgs.msg import Twist, PoseStamped
from rover.msg import StateMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tkinter import *   
import tkinter as tk



class createLabel:
    def __init__(self, queue, subType, initial, txt, x, y) -> None:
        """
        queue is the topic this label should subscribe to
        subType is the data type of the subscriber queue
        initial is the value we set the label to at the start
        txt is the string function the incoming msgs are formatted to
        x,y are the location on the screen
        """
        self.label = tk.Label(root, text=queue + " \n" + initial)
        self.queue = queue
        self.txt = txt
        self.subscriber = rospy.Subscriber(queue, subType, self.callback)
        self.label.place(x=x,y=y)
    
    def callback(self, data):
        # this updates the specific instance Label
        self.label['text'] = self.queue + "\n" + self.txt(data)

def twistStr(data):
    lin = "Linear Components: [x-%f, y-%f, z-%f]"%(data.linear.x, data.linear.y, data.linear.z)
    ang = "Angular Components: [x-%f, y-%f, z-%f]"%(data.angular.x, data.angular.y, data.angular.z)
    return lin + "\n" + ang

def poseStr(data):
    pos = "Positional Components: [x-%f, y-%f, z-%f]"%(data.pose.position.x, data.pose.position.y, data.pose.position.z)
    orn = "Orientation Components: [x-%f, y-%f, z-%f, w-%f]"%(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    return pos + "\n" + orn

def gpsStr(data):
    return "latitude: " + data.latitude + '\nlongitude:' + data.longitude



class interactStateMsg:
    def __init__(self, queue, label, initValue, choices, x, y) -> None:
        """
        queue is the name of the published topic
        type is the data type of the subscriber/publisher queue
        label is the value we set the label to at the start
        initValue is the value we set to the drop down initially
        choices are the options in the drop down
        x,y are the location on the screen
        """
        self.label = tk.Label(root, text=queue + " \n" + label)
        self.queue = queue
        self.subscriber = rospy.Subscriber('/rover_state', StateMsg, self.callback)
        self.label.place(x=x,y=y)
        self.stateMsg = -1

        self.pub = rospy.Publisher(queue, StateMsg, queue_size=1)
        self.clicked = StringVar(root)
        self.clicked.set(initValue)
        self.choices = choices
        self.drop = OptionMenu(root, self.clicked, *self.choices, command=self.updatePublisher)
        self.drop.place(x=x,y=y)
        root.after(1000, self.updatePublisher())

    def updatePublisher(self, *args):
        updatedMsg = self.stateMsg
        updatedMsg.rover_mode = self.clicked.get()
        self.pub.publish(updatedMsg)
    
    def callback(self, data):
        self.stateMsg = data
        self.label['text'] = self.queue + "\n" + self.stateStr()
    
    def stateStr(self):
        data = self.stateMsg
        st1 = "rover_mode: %f\nnext_goal_gps: %f\n"%(data.rover_mode, data.next_goal_gps)
        st2 = "current_task: %f\ngoal_gps_found: %f\n"%(data.current_task, data.goal_gps_found)
        st3 = "MISSION_OVER: %f\ndetected_AR_IDs: %f\n"%(data.MISSION_OVER, data.detected_AR_IDs)
        st4 = "curr_AR_IDs: %f\nbest_current_AR_ID: %f\n"%(data.curr_AR_IDs, data.best_current_AR_ID)
        return st1 + "\n" + st2 + "\n" + st3 + "\n" + st4



class commandButton():
    def __init__(self, name, action, x, y) -> None:
        """
        action is the command
        x,y are the location of the drop down on the screen
        """
        self.action = action
        self.btn = tk.Button(root, text = name, bd = '5', command = self.act)
        self.btn.place(x=x,y=y)

    def act(self):
        subprocess.call(self.action)



class createCamera:
    def __init__(self, queue, x, y) -> None:
        """
        queue is the name of the published topic
        x,y are the location of the camera feed on the screen
        """
        self.queue = queue
        self.feed = tk.Canvas(root, width=400, height=400, bd = 10, bg = 'white')
        self.feed.place(x=x,y=y)
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(queue, Image, self.callback)

    def callback(self, data):
        try:
          img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.feed.create_image(0,0, image=img)
        #self.feed.image = img




def main():
    rospy.init_node("gui")

    modes = ('AUTONOMY', 'MANUAL', 'IDLE')
    
    # create a tkinter window
    global root
    root = tk.Tk()             
    # Open window having dimension 100x100
    root.geometry('1000x1000')

    # create all the subscribers
    # createLabel('rover_gps', GPSMsg, "N/A", gpsStr, 100, 150)
    createLabel('cmd_vel', Twist, "N/A", twistStr, 100, 200)
    createLabel('move_base_simple/goal', PoseStamped, "N/A", poseStr, 100, 250)

    # create the label and drop down for StateMsg
    interactStateMsg('rover_state', "N/A", "IDLE", modes, 100, 400)

    # create terminal command
    commandButton("start ros bag", "rosbag record -a", 100, 600)

    # create camera feed
    # camera/color/image
    createCamera("/visualize", 500, 200)

    root.mainloop()

if __name__ == '__main__':
    main()