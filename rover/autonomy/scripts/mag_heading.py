import rospy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64
import math

class MagHeading:
    def __init__(self):
        self.mag_sub = rospy.Subscriber('/zed_node/imu/mag', MagneticField, self.mag_callback)
        self.mag_heading_pub = rospy.Publisher('/compass', Float64, queue_size=1)
        self.mag_heading = Float64()

    def mag_callback(self, msg):
        self.mag_heading.data = self.get_heading(msg.magnetic_field.x, msg.magnetic_field.y)

    def get_heading(self, x, y):
        heading = 0
        if x != 0:
            heading = math.atan2(y, x)
        return heading

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.mag_heading_pub.publish(self.mag_heading)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mag_heading')
    mag_heading = MagHeading()
    mag_heading.run()