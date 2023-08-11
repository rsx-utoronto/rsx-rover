#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from rover.msg import GPSMsg, StateMsg
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
from tkinter import *   
import tkinter as tk
from PIL import ImageTk, Image as ImagePIL



class createLabel:
    def __init__(self, queue, subType, initial, txt, x, y) -> None:
        """
        queue is the topic this label should subscribe to
        subType is the data type of the subscriber queue
        initial is the value we set the label to at the start
        txt is the string function the incoming msgs are formatted to
        x,y are the location on the screen
        """
        self.label = tk.Label(root, text=queue + " \n" + initial, justify='left')
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
    def __init__(self, sub, pub, label, initValue, choices, x, y) -> None:
        """
        sub is the name of the topic to listen to
        pub is the name of the topic to publish
        type is the data type of the subscriber/publisher topics
        label is the value we set the label to at the start
        initValue is the value we set to the drop down initially
        choices are the options in the drop down
        x,y are the location on the screen
        """
        self.label = tk.Label(root, text=sub + " \n" + label)
        self.sub = sub
        self.subscriber = rospy.Subscriber(sub, StateMsg, self.callback)
        self.label.place(x=x,y=y)
        self.stateMsg = -1

        self.pub = rospy.Publisher(pub, StateMsg, queue_size=10)
        self.clicked = StringVar(root)
        self.clicked.set(initValue)
        self.choices = choices
        self.drop = OptionMenu(root, self.clicked, *self.choices, command=self.updatePublisher)
        self.drop.place(x=x+300,y=y)
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
        st1 = "rover_mode: %f"%(data.rover_mode)
        st2 = "GPS_goals:\n%f"%(poseStr(data.GPS_goals))
        st3 = "curr_goal:\n%f"%(poseStr(data.curr_goals))
        st4 = "GPS_GOAL_REACHED: %f"%(data.GPS_GOAL_REACHED)
        st5 = "MISSION_OVER: %f"%(data.MISSION_OVER)
        st6 = "AR_TAG_DETECTED: %f"%(data.AR_TAG_DETECTED)
        st7 = "LIGHT_BEACON_DETECTED: %f"%(data.LIGHT_BEACON_DETECTED)
        st8 = "RADIO_BEACON_DETECTED: %f"%(data.RADIO_BEACON_DETECTED)
        st9 = "light_beacon_goal:\n%f"%(poseStr(data.light_beacon_goal))
        st10 = "radio_beacon_goal:\n%f"%(poseStr(data.radio_beacon_goal))
        st11 = "sign_goal:\n%f"%(poseStr(data.sign_goal))
        st12 = "curr_AR_ID: %f"%(data.curr_AR_ID)
        st13 = "curr_AR_pose:\n%f"%(poseStr(data.curr_AR_pose))
        st14 = "MANUAL_ENABLED:\n%f"%(data.MANUAL_ENABLED)
        comb1 = st1 + "\n" + st2 + "\n" + st3 + "\n" + st4 + "\n" + st5 + "\n" + st6 + "\n" + st7 + "\n" + st8
        comb2 = st9 + "\n" + st10 + "\n" + st11 + "\n" + st12 + "\n" + st13 + "\n" + st14
        return comb1 + "\n" + comb2



class createCamera:
    def __init__(self, queue, x, y, w, h) -> None:
        """
        queue is the name of the published topic
        x,y are the location of the camera feed on the screen
        w,h are the width and height of the image
        """
        self.queue = queue
        self.feed = tk.Canvas(root, width=w, height=h)
        self.feed.place(x=x,y=y)
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(queue, ImageMsg, self.callback, buff_size=2000000)
        self.w = w 
        self.h = h

    def callback(self, data):
        try:
          img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        img1 = ImagePIL.fromarray(img).resize((self.w, self.h))
        img2 = ImageTk.PhotoImage(image=img1)
        self.feed.create_image(0,0, image=img2)
        self.feed.image = img2
        #rospy.sleep(1)


class commandButton():
    def __init__(self, queue, name, comm, x, y) -> None:
        """
        x,y are the location of the drop down on the screen
        """
        self.comm = comm
        self.pub = rospy.Publisher(queue, String, queue_size=10)
        self.btn = tk.Button(root, text = name, bd = '5', command = self.act)
        self.btn.place(x=x,y=y)

    def act(self):
        self.pub.publish(self.comm)



def main():
    rospy.init_node("gui")

    modes = ('AUTONOMY', 'MANUAL', 'IDLE')
    
    # create a tkinter window
    global root
    root = tk.Tk()
    root.geometry('1350x750')

    # create terminal command
    # for this button to actually start a rosbag the script recordBag.py has to already be running
    commandButton("/gui_node/bagCommands", "start rosbag", "RECORD", 1000, 700)

    # create the label and drop down for StateMsg
    interactStateMsg('rover_state', "/gui_node/rover_state", "N/A", "IDLE", modes, 850, 150)

    # create camera feed
    # the first two numbers are x,y and the second two width, height
    createCamera("/camera/color/image_raw", 15, 15, 860, 640)

     # create all the subscribers
    createLabel('cmd_vel', Twist, "N/A", twistStr, 850, 15)
    createLabel('move_base_simple/goal', PoseStamped, "N/A", poseStr, 850, 75)

    # for arm team
    # not complete since I need the actual message types
    #createLabel('arm_safe_goal_pos', Twist, "N/A", twistStr, 15, 625)
    #createLabel('arm_error_msg', PoseStamped, "N/A", poseStr, 200, 625)
    #createLabel('arm_motor_curr', PoseStamped, "N/A", poseStr, 375, 625)
    #createLabel('arm_curr_pos', PoseStamped, "N/A", poseStr, 550, 625)

    root.mainloop()

if __name__ == '__main__':
    main()