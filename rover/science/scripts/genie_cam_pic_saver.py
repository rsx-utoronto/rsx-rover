import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class PicSaver:
    def __init__(self):
        self.pic_sub=rospy.Subscriber('geniecam', Image, self.pic_sub_callback)
        self.counter = 0
        self.bridge=CvBridge
        self.num_photos = 12 #change to predetermined number of photos!
        self.save_path = os.path.join(os.path.dirname(__file__), "calibrated_saved_images")
        os.makedirs(self.save_path, exist_ok=True)

    def pic_sub_callback(self, msg):
        if self.counter < self.num_photos:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg)
                filename = os.path.join(self.save_path, f"image_{self.counter:03d}.png")
                cv2.imwrite(filename, cv_image)
                self.counter += 1
            except Exception as e:
                rospy.logerr(f"Error saving image: {e}")

if __name__ == '__main__':
    rospy.init_node('pic_saver_node')
    ps = PicSaver()
    rospy.spin()
        
        
        
        
        