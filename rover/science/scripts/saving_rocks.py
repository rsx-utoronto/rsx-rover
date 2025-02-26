#!/usr/bin/env python3
import os
import serial
import time
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class CameraStoring:
    def __init__(self):
        self.start = False
        self.cam_data = None
        self.image = None
        self.bridge= CvBridge()

        self.camera_sub = rospy.Subscriber('geniecam', Image, callback = self.img_callback)
        self.GUI_sub = rospy.Subscriber('need_rocks', Bool, callback = self.starting_callback)

    # recieves a signal to start recording the camera data 
    def starting_callback(self, gui_data):
        if gui_data.data == True:
            self.start = True

    # recieves the current camera data 
    def img_callback(self, cam_data : Image):
        if self.start == True:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(cam_data, "8UC1")
                self.image = cv_image
                self.saving()
                
            except Exception as e:
                rospy.logerr(f"error: {e}")
        else:
            # rospy.loginfo("waiting for start command")
            pass

    # recieves information on what filter we are on 
    # CONFLICT OF INFORMATION: is science sending ros command for filter change, or is it software 


    # what information is the science team sending... how do i know that a filter has been switched?
    def filter_callback(self, filter_command):
        pass 

    def saving(self):
    # we have 1 picture (it is what the camera currently sees)
        print("saving")
        # not sure how the filter will be taken into account... for now im leaving it as this counter 
        filter = 12
        filter_count = 1

        image = 5

        # im not sure how to update the site yet. might be easier when we know how many sites we have and i can just add the number as a count 
        site = 1

        # NEED TO CREATE ROCK PHOTOS FOLDER IN THIS PATH 
        base_dir = "/home/rsx-base/rover_ws/src/rsx-rover/rover/science/genie_pics"
        folder_path = os.path.join(base_dir, f"folder_{site}")

        # change to whatever port it is 
        arduino_port = "/dev/ttyUSB0"  # CM0Must match the Arduino port
        baud_rate = 9600  # Must match the Arduino baud rate
        
        # creating serial port connection 
        ser = serial.Serial(arduino_port, baud_rate, timeout=1)
        time.sleep(2)  
        rate = rospy.Rate(0.2)
        while self.start and not rospy.is_shutdown():
            # create a folder, then store the images in this folder 1 time. then 
            
            #while 
            # while loop can end when counter is 12. also dont run when u dont get the S signal from ardiono 
                        
            for filter in range(filter):
                # create folder within the base_dir for the filter images
                # new_folder_path = os.path.join(folder_path, f"folder_{filter_count}")
                new_folder_path = folder_path
                if not os.path.exists(new_folder_path):
                    os.makedirs(new_folder_path)
                
                


                if self.image.all():
                     #cv2.imwrite("~/testing/image_name.jpeg", self.image)

                    img_path = os.path.join(new_folder_path, f"rock_image{filter_count}.jpeg")
                    #os.makedirs(img_path)

                    cv2.imwrite(img_path, self.image)
                    rospy.loginfo(f"image saved")

                    ser.write(b'M')  # Sending character 'M'
                    rospy.loginfo("Sent signal: M") 

                    pub = rospy.Publisher('need_rocks', Bool, queue_size=10)

                else:
                    # if there is no image saved (basically an error)
                    rospy.loginfo("waiting")

                '''while ser.read() != 'S':
                    rospy.loginfo("waiting for next filter")'''
                
                filter_count += 1
                rate.sleep()
                self.start = False 

            rate.sleep()

        # close serial port connection once we're done looking at the sites 
        ser.close()

if __name__ == '__main__':
    try:
        rospy.init_node('image_saving', anonymous=True)
        image_saving_node = CameraStoring()  
        rospy.spin()
    except rospy.ROSInterruptException:
        pass