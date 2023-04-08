#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool, Int64
from tkinter import *   
import tkinter as tk


class Label:
    labels = [] # this is a list of the created labels

    def __init__(self, queue, subType) -> None:
        """
        queue is the subscriber queue this label should follow
        subType is the data type of the subscriber queue
        """
        self.queue = queue
        self.pubType = subType
        self.label = tk.Label(root, text="placeholder")
        self.label.pack()
        Label.labels.append(self)
    
    def callback(self, data):
        # this updates the specific instance Label
        self.label['text'] = data.data

def updateLabels(): 
     # this updates all the Labels that have been created
    for label in Label.labels:
        rospy.Subscriber(label.queue, label.pubType, label.callback)
    root.after(100, lambda: Label.updateLabels())

class Button():
    def __init__(self, queue, pubType, varChoice1, varChoice2) -> None:
        """
        The button publishes to queue with data type pubType and toggles between the list varChoice1/2
        """
        self.pub = rospy.Publisher(queue, pubType, queue_size=1)
        self.var1 = pubType
        self.var2 = pubType
        self.curVar = pubType
        self.var1 = varChoice1
        self.var2 = varChoice2
        self.curVar = varChoice1
        self.btn = tk.Button(root, text = varChoice1, bd = '5', command = lambda: self.updatePublisher())
        self.btn.pack(side = 'top')   
        self.pub.publish(self.curVar)

    def updatePublisher(self):
        if self.curVar == self.var1:
            self.curVar = self.var2
        else:
            self.curVar = self.var1
        self.pub.publish(self.curVar)
        self.btn['text'] = self.curVar

def main():
    rospy.init_node("gui")
    
    # create a tkinter window
    global root
    root = tk.Tk()             
    # Open window having dimension 100x100
    root.geometry('300x300')

    # create all the labels
    Label('publish', Int64)
    Label('publish2', Int64)

    # create all the buttons
    Button('gui', Bool, True, False)

    updateLabels()
    root.mainloop()

if __name__ == '__main__':
    main()