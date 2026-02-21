import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import os
import time

class GenieCameraPublisher(Node): # for ros2 write "Node" inside brackets
    def __init__(self):
        """

        Initialize all camera values
        -----------------------------------------

        """
        super().__init__('genie_camera_publisher')
        print("genie_camera_publisher")

        # camera sensor properties
        width_max = 1936
        height_max = 1216
        binning = 0
        saturation = 0
        brightness = 0
        contrast = 0

        # desired properties
        crop_factor = 1.0
        self.width = int(width_max * 1/crop_factor)
        self.height = int(height_max * 1/crop_factor)
        x_offset = int((width_max - self.width) / 2)
        y_offset = int((height_max - self.height) / 2)

        self.ctx.GevSetImageParameters(self.width,
                                self.height,
                                x_offset,
                                y_offset,
                                params['pixelFormat'][0])
        params = self.ctx.GevGetImageParameters()
        print("Final image parameters:")
        print(params)

        self.width = params['width']
        self.height = params['height']

        self.take_image_sub = self.create_subscription( String, "/save_genie_image",self.save_genie_callback, 10)

        

    def publish_ros_topic(self):
        """
        Continuously publish the image to a ROS topic.
        -----------------------------------------
        """

        # rospy.init_node("geniecam")

       
        pub = self.create_publisher( Image, "geniecam", 10) # uses Image

        bridge = CvBridge()

        while not rclpy.ok(): 

            img = self._get_image()
            self.img = bridge.cv2_to_imgmsg(img, "8UC1")
            pub.publish(self.img) 

            time.sleep(1/2)

    def save_genie_callback(self, msg):
        msgs = msg.data.split(",")
        try:
            # Create bridge to convert ROS Image to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
            
            # Create directory if it doesn't exist
            images_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data", "genie_images", msgs[0])
            if not os.path.exists(images_dir):
                os.makedirs(images_dir)
            
            # Create a timestamp for a unique filename
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(images_dir, f"genie_image_{msgs[1]}.png")
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Notify user
            print(f"Genie image saved to {filename}")
            
        except Exception as e:
            print(f"Error saving genie image: {e}")  




if __name__ == "__main__":
    # rclpy.init()
    try:
        g = GenieCameraPublisher()
        
        # while rclpy.ok() and not g.camera_found:
        #     print("Waiting for genie camera...")
        #     time.sleep(2)

        if g.camera_found:
            print("Genie camera found!")
            #g.publish_ros_topic()
            filename = input("Enter image name: ")
            g.capture_image(filename + ".jpg")  
        else:
            print("No Genie camera found.")

    except: # rclpy.exceptions.ROSInterruptException:
        print("life sucks")