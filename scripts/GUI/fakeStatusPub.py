#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class FakeStatusPublisher(Node):

    def __init__(self):
        super().__init__('fake_status_publisher')
        self.publisher_ = self.create_publisher(String, 'gui_status', 10)
        self.counter = 0
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        if self.counter % 10 == 0:
            msg = String()
            msg.data = 'Goal Point Reached: GNSS1'
        else:
            msg = String()
            msg.data = f'Fake status message {time.time()}'

        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3

# import rclpy 
# from rclpy.node import node
# from std_msgs.msg import String

# def fake_status_publisher():
#     rospy.init_node('fake_status_publisher', anonymous=True)
#     pub = rospy.Publisher('gui_status', String, queue_size=10)
#     rate = rospy.Rate(1)  # 1 Hz
#     i = 0
#     while not rospy.is_shutdown():
#         status_message = "Fake status message" + str(rospy.get_time())
#         rospy.loginfo(status_message)
#         rate.sleep()
#         i += 1
#         if i % 10 == 0:
#             pub.publish("Goal Point Reached: GNSS1")
#         else:
#             pub.publish(status_message)
            
# if __name__ == '__main__':
#     try:
#         fake_status_publisher()
#     except rospy.ROSInterruptException:
#         pass