#!/usr/bin/python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float64MultiArray

class Aimer:
    def __init__(self, frame_width: int, frame_height: int, min_aruco_area: float, 
                 aruco_min_x_uncert: float, aruco_min_area_uncert: float,
                 max_linear_v: float, max_angular_v: float) -> None:
        self.target_x = frame_width / 2
        self.target_y = frame_height / 2
        self.max_linear_v = max_linear_v
        self.max_angular_v = max_angular_v
        self.min_aruco_area = min_aruco_area
        self.aruco_min_x_uncert = aruco_min_x_uncert
        self.aruco_min_area_uncert = aruco_min_area_uncert
        self.linear_pid = PID(0.1, 0.01, 0.01)  # Change PID constants as needed
        self.angular_pid = PID(0.1, 0.01, 0.01)  # Change PID constants as needed
        self.linear_v = None
        self.angular_v = None

    def update(self, aruco_top_left: tuple, aruco_top_right: tuple, 
               aruco_bottom_left: tuple, aruco_bottom_right: tuple) -> None:
        if None in (aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right):
            self.linear_v, self.angular_v = 0, 0
            return

        aruco_x = (aruco_top_left[0] + aruco_top_right[0] + aruco_bottom_left[0] + aruco_bottom_right[0]) / 4
        if abs(aruco_x - self.target_x) > self.aruco_min_x_uncert:
            if aruco_x < self.target_x:
                out_angular = 1
            else:
                out_angular = -1
        else:
            out_angular = 0
            self.angular_pid.reset()
        
        aruco_distance_est = self.calculate_area(aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right)
        if aruco_distance_est < self.min_aruco_area:
            out_linear = 1
        else:
            out_linear = 0
            self.linear_pid.reset()

        self.linear_v, self.angular_v = out_linear, out_angular

    def calculate_area(self, aruco_top_left: tuple, aruco_top_right: tuple, 
                       aruco_bottom_left: tuple, aruco_bottom_right: tuple) -> float:
        diag1 = np.array(aruco_top_right) - np.array(aruco_bottom_left)
        diag2 = np.array(aruco_top_right) - np.array(aruco_bottom_right)
        diag3 = np.array(aruco_top_right) - np.array(aruco_top_left)

        cross_product_1 = np.cross(diag1, diag2)
        cross_product_2 = np.cross(diag1, diag3)

        parallelogram_area = np.linalg.norm(cross_product_1) + np.linalg.norm(cross_product_2)
        quadrilateral_area = parallelogram_area / 2

        return quadrilateral_area


class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.error_prev = 0
        self.error_sum = 0

    def update(self, error: float) -> float:
        self.error = error
        self.error_sum += error
        output = self.kp * error + self.ki * self.error_sum + self.kd * (error - self.error_prev)
        self.error_prev = error
        return output
    
    def set_constants(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self) -> None:
        self.error_sum = 0


class AimerROS(Aimer):
    def __init__(self, frame_width: int, frame_height: int, min_aruco_area: float, 
                 aruco_min_x_uncert: float, aruco_min_area_uncert: float,
                 max_linear_v: float, max_angular_v: float) -> None:
        super().__init__(frame_width, frame_height, min_aruco_area, aruco_min_x_uncert, aruco_min_area_uncert, max_linear_v, max_angular_v)

    def rosUpdate(self, data: Int32MultiArray) -> None:
        aruco_top_left = (data.data[0], data.data[1])
        aruco_top_right = (data.data[2], data.data[3])
        aruco_bottom_left = (data.data[4], data.data[5])
        aruco_bottom_right = (data.data[6], data.data[7])
        self.update(aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right)


def main():
    rospy.init_node('aruco_homing', anonymous=True)
    pub = rospy.Publisher('drive', Twist, queue_size=10)
    aimer = AimerROS(640, 360, 1000, 100, 100, 0.5, 0.5)
    rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist = Twist()
        if aimer.linear_v == 0 and aimer.angular_v == 0:
            twist.linear.x = 0
            twist.angular.z = 0
        elif aimer.angular_v == 1:
            twist.angular.z = aimer.max_angular_v
            twist.linear.x = 0
        elif aimer.angular_v == -1:
            twist.angular.z = -aimer.max_angular_v
            twist.linear.x = 0
        elif aimer.linear_v == 1:
            twist.linear.x = aimer.max_linear_v
            twist.angular.z = 0

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()

aimer = AimerROS(1133, 378, 1000, 100, 100, 0.5, 0.5) # change constants
    #waterbottle 21.5 cm (812) tall 9 cm (340) diameter 