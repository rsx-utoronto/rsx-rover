#!/usr/bin/python3

import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float64MultiArray

class Aimer: # 

    # frame_width: the width of the frame in pixels
    # frame_height: the height of the frame in pixels
    # min_aruco_area: the minimum area of the aruco marker in pixels when the robot is at the desired distance from the marker, or closer
    # aruco_min_x_uncert: the maximum distance the aruco marker can be from the center of the frame in pixels (for calculations about turning)
    # aruco_min_area_uncert: the maximum difference between the actual area of the aruco marker and the minimum area of the aruco marker in pixels (for calculations about moving forward)

    def __init__(self, frame_width: int, frame_height: int, min_aruco_area: float, 
                 aruco_min_x_uncert: float, aruco_min_area_uncert: float,
                 max_linear_v: float, max_angular_v: float
                 ) -> None:
        self.target_x = frame_width/2
        self.target_y = frame_height/2
        self.max_linear_v = max_linear_v
        self.max_angular_v = max_angular_v
        self.min_aruco_area = min_aruco_area
        self.aruco_min_x_uncert = aruco_min_x_uncert
        self.aruco_min_area_uncert = aruco_min_area_uncert
        self.linear_pid = PID(0.1, 0.01, 0.01) # change pid constants
        self.angular_pid = PID(0.1, 0.01, 0.01) # change pid constants
        self.linear_v = None
        self.angular_v = None

    def update(self, aruco_top_left: tuple, aruco_top_right: tuple, 
               aruco_bottom_left: tuple, aruco_bottom_right: tuple) -> None: # update linear_v, angular_v
        # if aruco_top_left == None or aruco_top_right == None or aruco_bottom_left == None or aruco_bottom_right == None:
        #    self.linear_v, self.angular_v = 0, 0

        # the middle of the aruco tag is aruco_x
        aruco_x = (aruco_top_left[0] + aruco_top_right[0] + aruco_bottom_left[0] + aruco_bottom_right[0]) / 4
        print ("\n\ntuple stuff")
        print (aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right)
        print("aruco_x", aruco_x)
        print("first if statement", abs(aruco_x - self.target_x), "should be larger than", self.aruco_min_x_uncert)
        if abs(aruco_x - self.target_x) > self.aruco_min_x_uncert:
            if aruco_x < self.target_x:
                out_angular = 1
            else:
                out_angular = -1 # self.angular_pid.update(self.target_x - aruco_x)
        else:
            print ("at no turning\n\n")
            out_angular = 0
            self.angular_pid.reset()
        
        aruco_distance_est = self.calculate_area(aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right)
        print("aruco_distance_est", aruco_distance_est)
        print("second if statement", abs(aruco_distance_est - self.min_aruco_area), "should be larger than", self.aruco_min_area_uncert)

        if aruco_distance_est < self.min_aruco_area:
            # if negative, then move forward
            print ("LINEAR: too far")
            out_linear = 1 
        else:
            print ("LINEAR: do not go forward")
            out_linear = 0
            self.linear_pid.reset()

        self.linear_v, self.angular_v = out_linear, out_angular # min(self.max_linear_v, out_linear), min(self.max_angular_v, out_angular)

    def calculate_area(self, aruco_top_left: tuple, aruco_top_right: tuple, 
                       aruco_bottom_left: tuple, aruco_bottom_right: tuple) -> float:
        # calculate area of quadrilateral using cross products
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

class AimerROS(Aimer):  #updates coords continuously 
    def __init__(self, frame_width: int, frame_height: int, min_aruco_area: float, 
                 aruco_min_x_uncert: float, aruco_min_area_uncert: float,
                 max_linear_v: float, max_angular_v: float) -> None:
        super().__init__(frame_width, frame_height, min_aruco_area, aruco_min_x_uncert, aruco_min_area_uncert, max_linear_v, max_angular_v)
        
    def rosUpdate(self, data: Int32MultiArray) -> None:
        print ("\nDATA FROM AIMER ", data)
        aruco_top_left = (data.data[0], data.data[1])
        aruco_top_right = (data.data[2], data.data[3])
        aruco_bottom_left = (data.data[4], data.data[5])
        aruco_bottom_right = (data.data[6], data.data[7])
        self.update(aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right)

def main():
    pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
    # frame_width, frame_height, min_aruco_area, aruco_min_x_uncert, aruco_min_area_uncert, max_linear_v, max_angular_v
    aimer = AimerROS(640, 360, 1000, 100, 100, 1.8, 0.8) # FOR ARUCO
    
    # aimer = AimerROS(640, 360, 1450, 50, 200, 1.0, 0.5) # FOR WATER BOTTLE
    rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate) # change topic name
    # int32multiarray convention: [top_left_x, top_left_y, top_right_x, top_right_y, bottom_left_x, bottom_left_y, bottom_right_x, bottom_right_y]
    rate = rospy.Rate(10)
    prev_flag =  ""
    flag = ""
    startRotationTime = time.time()
    while not rospy.is_shutdown():
        twist = Twist()
        if(time.time()-startRotationTime) > 35:
            print ("failure", aimer.linear_v, aimer.angular_v)
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            return False
        if aimer.linear_v == 0 and aimer.angular_v == 0:
            print ("at weird", aimer.linear_v, aimer.angular_v)
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            return True
        if aimer.angular_v == 1:
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
    rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
    main()