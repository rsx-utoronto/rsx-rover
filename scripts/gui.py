#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Bool 


# import everything from tkinter module
from tkinter import *   






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