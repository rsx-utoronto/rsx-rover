#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
#from rsx_rover.msg import GPSMsg, StateMsg
from sensor_msgs.msg import Image
from tkinter import *   
import tkinter as tk



class Label:
    def __init__(self, queue, subType, initial, txt, x, y) -> None:
        """
        queue is the subscriber queue this label should follow
        subType is the data type of the subscriber queue
        initial is the value we set the label to at the start
        txt is the string function the incoming published msgs are formatted to
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

def stateStr(data):
    st1 = "next_goal_gps: %f\ncurrent_task: %f\ngoal_gps_found: %f"%(data.next_goal_gps, data.current_task, data.goal_gps_found)
    st2 = "MISSION_OVER: %f\ndetected_AR_IDs: %f\n"%(data.MISSION_OVER, data.detected_AR_IDs)
    st3 = "curr_AR_IDs: %f\nbest_current_AR_ID: %f\n"%(data.curr_AR_IDs, data.best_current_AR_ID)
    return st1 + "\n" + st2 + "\n" + st3

def gpsStr(data):
    return "latitude: " + data.latitude + '\nlongitude:' + data.longitude



class DropDownPub():
    def __init__(self, queue, pubType, initial, choices, x, y) -> None:
        """
        queue is the name of the published topic
        pubType is the data type of the publisher
        choices are the options in the drop down
        x,y are the location of the drop down on the screen
        """
        self.pub = rospy.Publisher(queue, pubType, queue_size=1)
        self.clicked = StringVar(root)
        self.clicked.set(initial)
        self.type = pubType
        self.choices = choices
        self.drop = OptionMenu(root, self.clicked, *self.choices, command=self.updatePublisher)
        self.drop.place(x=x,y=y)
        root.after(self.updatePublisher())

    def updatePublisher(self, *args):
        var = self.type()
        var = self.clicked.get()
        print(var)
        print(type(var))
        self.pub.publish(var)

class Button():
    def __init__(self, name, action, x, y) -> None:
        """
        queue is the name of the published topic
        pubType is the data type of the publisher
        choices are the options in the drop down
        x,y are the location of the drop down on the screen
        """
        self.action = action
        self.btn = tk.Button(root, text = name, bd = '5', command = self.updatePublisher)
        self.btn.place(x=x,y=y)
        self.updatePublisher()

    def updatePublisher(self):
        pass




def main():
    rospy.init_node("gui")

    modes = ('AUTONOMY', 'MANUAL', 'IDLE')
    
    # create a tkinter window
    global root
    root = tk.Tk()             
    # Open window having dimension 100x100
    root.geometry('1000x1000')

    # create all the subscribers
    #Label('rover_state', StateMsg, "N/A", stateStr, 100, 50)
    #Label('rover_gps', GPSMsg, "N/A", gpsStr, 100, 150)
    Label('cmd_vel', Twist, "N/A", twistStr, 100, 200)
    Label('move_base_simple/goal', PoseStamped, "N/A", poseStr, 100, 250)

    # create all the publishers
    DropDownPub('rover/launch', String, "IDLE", modes, 100, 400)

    # create terminal command
    #Button("start ros bag", "rosbag record -a", 100, 600)

    root.mainloop()

if __name__ == '__main__':
    main()