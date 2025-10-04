#!/usr/bin/python3

import numpy as np
import rclpy 
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float64MultiArray
import yaml

file_path = "/home/rsx-base/rover_ws/src/rsx-rover/rover/autonomy/scripts/sm_config.yaml" #Need a better way to do this, fine for testing

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

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
        self.aruco_top_left = None
        self.aruco_top_right = None
        self.aruco_bottom_left = None
        self.aruco_bottom_right = None

    def update(self, aruco_top_left: tuple, aruco_top_right: tuple,  #determines what the speeds should be
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
    
    #scaling velocity code, when aruco_distance_est near 0, velocity near 1, when aruco_distance_est near min_aruco_area, velocity equals 0
    #def scale_velocity(self, aruco_distance_est: float):
    #    velocity = int((5*(self.min_aruco_area - aruco_distance_est)) / self.min_aruco_area)
    #    velocity = velocity / 5
    #    
    #    return velocity
    


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
        
    def rosUpdate(self, data: Int32MultiArray) -> None: #Callback function for recieving data of the bbox
        print ("\nDATA FROM AIMER ", data)
        self.aruco_top_left = (data.data[0], data.data[1])
        self.aruco_top_right = (data.data[2], data.data[3])
        self.aruco_bottom_left = (data.data[4], data.data[5])
        self.aruco_bottom_right = (data.data[6], data.data[7])
        #self.update(aruco_top_left, aruco_top_right, aruco_bottom_left, aruco_bottom_right) #Have to take this out
        


class ArucoHomingNode(Node):
    def __init__(self):
        super().__init__('aruco_homing')
        # Configuration parameters
        frame_width = 640
        frame_height = 360
        min_aruco_area = 1000.0
        aruco_min_x_uncert = 100.0
        aruco_min_area_uncert = 100.0
        max_linear_v = 1.8
        max_angular_v = 0.8
        
        self.aimer = AimerROS(frame_width, frame_height, min_aruco_area,
                             aruco_min_x_uncert, aruco_min_area_uncert,
                             max_linear_v, max_angular_v)
        
        # ROS 2 setup
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'aruco_node/bbox',
            self.aimer.rosUpdate,
            10)
        
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.active = True
    def timer_callback(self):
        if not self.active:
            return
            
        # Timeout check
        if time.time() - self.start_time > 35:
            self.get_logger().error("Timeout (35s) reached")
            self.stop_robot()
            self.active = False
            return

        # Success check
        if self.aimer.linear_v == 0 and self.aimer.angular_v == 0:
            self.get_logger().info("Target reached")
            self.stop_robot()
            self.active = False
            return

        # Control logic
        twist = Twist()
        if self.aimer.angular_v == 1:
            twist.angular.z = self.aimer.max_angular_v
        elif self.aimer.angular_v == -1:
            twist.angular.z = -self.aimer.max_angular_v
        elif self.aimer.linear_v == 1:
            twist.linear.x = self.aimer.max_linear_v
            
        self.publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.publisher.publish(twist)    

def main(args=None): #Assuming that we already detect aruco/waterbottle/mallet
   
    rclpy.init(args=args)
    node = ArucoHomingNode()
    rclpy.spin(node) #main code should come after this
    
    #Main testing code
    pub = rclpy.create_publisher(Twist, 'drive', 10) # change topic name
    
    #frame_width, frame_height, min_aruco_area, aruco_min_x_uncert, aruco_min_area_uncert, max_linear_v, max_angular_v
    if sm_config.get("realsense_detection"):
        aimer = AimerROS(640, 360, 2500, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
    else: #For zed camera
        aimer = AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO

    #aimer = aruco_homing.AimerROS(640, 360, 1450, 100, 200, sm_config.get("Obj_homing_lin_vel"), sm_config.get("Obj_homing_ang_vel")) # FOR WATER BOTTLE
    #self.create_subscription(Float64MultiArray, 'object/bbox', aimer.rosUpdate, 10) #For waterbottle
    rclpy.create_subscription(Float64MultiArray,'aruco_node/bbox', aimer.rosUpdate, 10) # change topic name
    #int32multiarray convention: [top_left_x, top_left_y, top_right_x, top_right_y, bottom_left_x, bottom_left_y, bottom_right_x, bottom_right_y]
    rate = rclpy.Rate(10)
    #prev_flag =  ""
    #flag = ""
    startRotationTime = time.time()
    last_detection_time = time.time()
    detection_memory_duration = 2.0  # 2 seconds of memory
    detection_active = False

    while (rclpy.ok()):
        twist = Twist()
        aimer.update(aimer.aruco_top_left, aimer.aruco_top_right, aimer.aruco_bottom_left, aimer.aruco_bottom_right)

        # Check if we have valid values from the aimer
        if aimer.linear_v is not None and aimer.angular_v is not None:
            # We have a detection, update the timer
            last_detection_time = time.time()
            detection_active = True
            
            # Check if we've reached the target
            if aimer.linear_v == 0 and aimer.angular_v == 0:
                print ("at weird", aimer.linear_v, aimer.angular_v)
                if first_time:
                    first_time=False
                    initial_time=time.time()
                
                # faking it til you makin it
                while abs(initial_time-time.time()) < 0.7: #Is it ok? Can switch to commented out main if not
                    twist.linear.x= sm_config.get("GS_Traversal_lin_vel")
                    twist.angular.z= 0
                    pub.publish(twist) #drive publisher
                    print("final homing movement",abs(initial_time-time.time()) )
                    rclpy.timer.Rate(1).sleep()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                
                return
                
            # Normal homing behavior
            if aimer.angular_v == 1:
                twist.angular.z = float(aimer.max_angular_v)
                print ("firstd if",aimer.max_angular_v)
                twist.linear.x = 0.0
            elif aimer.angular_v == -1:
                twist.angular.z = float(-aimer.max_angular_v)
                twist.linear.x = 0.0
            elif aimer.linear_v == 1:
                print ("second check",aimer.max_linear_v)
                twist.linear.x = float(aimer.max_linear_v)
                twist.angular.z = 0.0
            
            last_detectionp_linear_velocity = float(twist.linear.x) #this line to be used when we are scaling the speed.
            last_detectionp_angular_velocity = float(twist.angular.z) #this line to be used when we are scaling the speed.

            #last_detectionp_aruco_tl = aimer.aruco_top_left
            #last_detectionp_aruco_tr = aimer.aruco_top_right
            #last_detectionp_aruco_br = aimer.aruco_bottom_right
            #last_detectionp_aruco_bl = aimer.aruco_bottom_left
            #can be used in the future

        else:
            # No detection, check if we're within memory duration
            if detection_active and time.time() - last_detection_time < detection_memory_duration:
                twist.linear.x = last_detectionp_linear_velocity * -1
                twist.angular.z = last_detectionp_angular_velocity * -1
                print("Going back or turning the angle back when no detection and still in within the memory duration") 
            else:
                # Memory expired, go back to grid search
                print("Detection lost and memory expired, returning to grid search")
                detection_active = False
                break
        
            
        pub.publish(twist)
        rclpy.timer.Rate(1).sleep()


    """ while rclpy.ok():
        aimer.update(aimer.aruco_top_left, aimer.aruco_top_right, aimer.aruco_bottom_left, aimer.aruco_bottom_right)
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
        rate.sleep() """

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
    # rclpy.init()
    # node='aruco_homing'
    
    main()