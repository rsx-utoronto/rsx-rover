#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
from tkinter import *   
import tkinter as tk
from PIL import ImageTk, Image as ImagePIL
from rover.msg import StateMsg
import sensor_msgs
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from tf.transformations import euler_from_quaternion



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
        self.stateMsg = StateMsg()

        self.pub = rospy.Publisher(pub, StateMsg, queue_size=10)
        self.clicked = StringVar(root)
        self.clicked.set(initValue)
        self.choices = choices
        self.drop = OptionMenu(root, self.clicked, *self.choices, command=self.updatePublisher)
        self.drop.place(x=x-50,y=y)
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
        (curr_r,curr_p,curr_y) = euler_from_quaternion([data.curr_goal.pose.orientation.x, data.curr_goal.pose.orientation.y, data.curr_goal.pose.orientation.z, data.curr_goal.pose.orientation.w])
        (l_r,l_p,l_y) = euler_from_quaternion([data.light_beacon_goal.pose.orientation.x, data.light_beacon_goal.pose.orientation.y, data.light_beacon_goal.pose.orientation.z, data.light_beacon_goal.pose.orientation.w])
        (r_r,r_p,r_y) = euler_from_quaternion([data.radio_beacon_goal.pose.orientation.x, data.radio_beacon_goal.pose.orientation.y, data.radio_beacon_goal.pose.orientation.z, data.radio_beacon_goal.pose.orientation.w])
        (s_r,s_p,s_y) = euler_from_quaternion([data.sign_goal.pose.orientation.x, data.sign_goal.pose.orientation.y, data.sign_goal.pose.orientation.z, data.sign_goal.pose.orientation.w])
        st1 = f"rover_mode: {data.rover_mode}\n"
        st2 = f"ar_tag_detected: {data.AR_TAG_DETECTED}\nlight_beacon_detected: {data.LIGHT_BEACON_DETECTED}\nradio_beacon_detected: {data.RADIO_BEACON_DETECTED}\n"
        st3 = f"light_beacon_goal: x - {data.light_beacon_goal.pose.position.x}  y - {data.light_beacon_goal.pose.position.y} w - {l_y}\n"
        st4 = f"radio_beacon_goal: x - {data.radio_beacon_goal.pose.position.x}  y - {data.radio_beacon_goal.pose.position.y} w - {r_y}\n"
        st5 = f"sign_goal: x - {data.sign_goal.pose.position.x}  y - {data.sign_goal.pose.position.y} w - {s_y}\n"
        st6 = f"Curr_AR_ID: {data.curr_AR_ID}\n"
        st7 = f"gps_goal_reached: {data.GPS_GOAL_REACHED}\nMission over: {data.MISSION_OVER}\n"
        return st1 + "\n" + st2 + "\n" + st3 + "\n" + st4 + "\n" + st5 + "\n" + st6 + "\n" + st7



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
    # Open window having dimension 100x100
    root.geometry('1000x1000')

    # create all the subscribers
    createLabel('rover_gps', NavSatFix, "N/A", gpsStr, 100, 150)
    createLabel('drive', Twist, "N/A", twistStr, 100, 200)
    createLabel('move_base_simple/goal', PoseStamped, "N/A", poseStr, 100, 250)

    # create the label and drop down for StateMsg
    interactStateMsg('rover_state', "N/A", "IDLE", modes, 150, 400)

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