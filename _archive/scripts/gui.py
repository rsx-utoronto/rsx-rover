#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Bool 


# import everything from tkinter module
from tkinter import *   


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
    pub_gui = rospy.Publisher('gui', Bool, queue_size=100)
    rospy.init_node("gui")
    v = Bool()

    # create a tkinter window
    root = Tk()             
    
    # Open window having dimension 100x100
    root.geometry('100x100')
    
    # Create a Button
    btn = Button(root, text = 'Click me !', bd = '5',
                        command = root.destroy)
    
    # Set the position of button on the top of window.  
    btn.pack(side = 'top')   
    print('we made a screen')
    while not rospy.is_shutdown():
        v = True
        pub_gui.publish(v)
        print('we published')
        root.mainloop()
        print('out of tk loop')
        time.sleep(1) # this is the amount of time (between loop iterations

if __name__ == '__main__':
    main()