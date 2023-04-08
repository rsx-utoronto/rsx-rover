#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool, Int64
from tkinter import *   
import tkinter as tk


class Label:
    labels = [] # this is a list of the created labels

    def __init__(self, queue, pubType) -> None:
        """
        queue is the subscriber queue this label should follow
        pubType is the data type of the subscriber queue
        """
        self.queue = queue
        self.pubType = pubType
        self.label = Label(root, text="placeholder")
        self.label.pack()
        Label.labels.append(self)
    
    def updateLabels(): 
        # this updates all the Labels that have been created
        for label in Label.labels:
            rospy.Subscriber(label.queue, label.pubType, label.callback)
        root.after(100, Label.updateLabels())
    
    def callback(self, data):
        # this updates the specific instance Label
        self.label['text'] = data.data

"""
def updatePublisher(b):
    global v
    v = not b
    pub_gui.publish(v)
    print('here')
"""

def main():
    #global pub_gui
    #pub_gui = rospy.Publisher('gui', Bool, queue_size=100)
    rospy.init_node("gui")
    
    # create a tkinter window
    global root
    root = tk.Tk()             
    # Open window having dimension 100x100
    root.geometry('200x200')

    # create all the labels
    lab = Label('publish', Int64)

    """
    global v
    v = Bool()
    v = True
    
    # Create a Button
    btn = Button(root, text = 'Click me !', bd = '5',
                        command = lambda: updatePublisher(v))
    
    # Set the position of button on the top of window.  
    btn.pack(side = 'top')   
    pub_gui.publish(v)
    """

    Label.updateLabels()
    root.mainloop()

if __name__ == '__main__':
    main()