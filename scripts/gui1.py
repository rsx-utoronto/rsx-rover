#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool, Int64, Float64
from tkinter import *   
import tkinter as tk


class Label:
    labels = [] # this is a list of the created labels

    def __init__(self, queue, subType, initial, x, y) -> None:
        """
        queue is the subscriber queue this label should follow
        subType is the data type of the subscriber queue
        initial is the value we set the label to at the start
        """
        self.label = tk.Label(root, text=initial)
        self.subscriber = rospy.Subscriber(queue, subType, self.callback)
        self.label.place(x=x,y=y)
    
    def callback(self, data):
        # this updates the specific instance Label
        self.label['text'] = data.data

class Button():
    def __init__(self, queue, pubType, varChoice1, varChoice2, x, y) -> None:
        """
        The button publishes to queue with data type pubType and toggles between the list varChoice1/2.
        The text displayed on the button is the current value being published.
        """
        self.pub = rospy.Publisher(queue, pubType, queue_size=1)
        self.var1 = pubType
        self.var2 = pubType
        self.curVar = pubType
        self.var1 = varChoice1
        self.var2 = varChoice2
        self.curVar = varChoice1
        self.btn = tk.Button(root, text = varChoice1, bd = '5', command = lambda: self.updatePublisher())
        self.btn.place(x=x,y=y)
        self.updatePublisher()

    def updatePublisher(self):
        if self.curVar == self.var1:
            self.curVar = self.var2
        else:
            self.curVar = self.var1
        self.pub.publish(self.curVar)
        self.btn['text'] = self.curVar

class Slider():
    def __init__(self, queue, pubType, val, res, min, max, orn, initial, x, y) -> None:
        """
        The slider publishes to queue with data type pubType and has a range from min to max. The 
        orientation of the slider (orn) is either HORIZONTAL or VERTICAL. Sliders can have either
        return values as IntVar, DoubleVar or StringVar which is set by val. The resolution step between 
        options on the slider (0.2 would mean the scale has 6 value options from 0.0 to 1.0) and is set 
        by the variable res. The starting value of the slider is initial
        """
        if orn == VERTICAL:
            min, max = max, min
        self.pub = rospy.Publisher(queue, pubType, queue_size=1)
        self.type = val
        self.scale = tk.Scale(root,from_=min,to=max,orient=orn,
                              resolution=res)
        self.scale.set(initial)   
        self.var = initial
        self.scale.bind("<ButtonRelease-1>", lambda x: self.updatePublisher())
        self.scale.place(x=x, y=y)
        self.updatePublisher()

    def updatePublisher(self):
        print('here')
        print(self.var)
        if self.type == float:
            self.var = float(self.scale.get())
        elif self.type == int:
            self.var = int(self.scale.get())
        else:
            self.var = str(self.scale.get())
        print(self.var)
        self.pub.publish(self.var)

def main():
    rospy.init_node("gui")
    
    # create a tkinter window
    global root
    root = tk.Tk()             
    # Open window having dimension 100x100
    root.geometry('500x500')

    # create all the labels
    Label('publish', Int64, 0, 100, 100)
    Label('publish2', Int64, 10000, 100, 150)

    # create all the buttons
    Button('gui', Bool, True, False, 100, 400)

    # create all the sliders
    Slider('slider', Float64, float, 0.2, 0.0, 1.0, VERTICAL, 1.0, 300, 300)
    Slider('slider2', Float64, float, 0.2, 0.0, 1.0, HORIZONTAL, 1.0, 330, 400)

    root.mainloop()

if __name__ == '__main__':
    main()