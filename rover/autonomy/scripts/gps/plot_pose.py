#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import time
# import tf
# import tf_transformations 
# import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# import numpy.typing as npt
import statistics as stats
import math
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class PlotPose(Node):
    """
    A class for visualizing the position of a robot in real-time.

    This class subscribes to pose and odometry topics in ROS, collects position data, and
    generates live plots using Matplotlib. The position data is plotted in the XY plane,
    providing a visual representation of the robot's movement over time.
    """
    def __init__(self):
        """
        Initializes the PlotPose class by setting up ROS subscribers, Matplotlib figures, and
        buffers for storing positional data.

        Attributes:
            fig_xy (matplotlib.figure.Figure): The figure for plotting the XY position of the robot.
            ax_xy (matplotlib.axes.Axes): The axes associated with the XY figure.
        """
        super().__init__('plot_pose')

        # Initialize figure and axes and save to class
        self.fig_xy, self.ax_xy = plt.subplots()
        # self.fig_err, self.ax_err = plt.subplots()
        # self.fig_x, self.ax_x = plt.subplots()
        # self.fig_y, self.ax_y = plt.subplots()
        # print("Initializing plot")

        # self.ax.legend()
        # create Thread lock to prevent multiaccess threading errors
        # self._lock = threading.Lock()

        
        self.init_time = time.time()
        self.pose_subscriber = self.create_subscription( PoseStamped,'/pose', self.pose_callback,10)

        self.odom_subscriber = self.create_subscription( Odometry, '/odometry/filtered', self.odom_callback,10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.x_list = [] # List of x positions
        self.y_list = [] # List of y positions 

        # # For getting data from tf2_ros
        # self.v_x_list = [] # List of x positions from vicon
        # self.v_y_list = [] # List of y positions from vicon
        # self.v_x_t_list = [] # List of x positions from the tf listener (vicon)
        # self.v_y_t_list = [] # List of y positions from the tf listener (vicon)
        # self.v_exist = False

        self.o_x_list = [] # List of x positions from odom
        self.o_y_list = [] # List of y positions from odom

        # self.tf2_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf2_buffer)
    def pose_callback(self, msg):

        # rospy.loginfo("Received a new pose from cartographer")
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.orient_x = msg.pose.orientation.x
        self.orient_y = msg.pose.orientation.y
        self.orient_z = msg.pose.orientation.z
        self.orient_w = msg.pose.orientation.w
        self.stamp = msg.header.stamp

        self.x_list.append(self.x)
        self.y_list.append(self.y)

        ############################# Getting the transform between odom and base_link using tf2_ros #############################

        # We don't this way of getting the transform because this is an old method which does not give all the information (like we don't get a timestamp for the transform)
        # try:
        #     (self.v_trans, self.v_rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time()) 
            
        #     # tried rospy.Time(0) but it does not solve the issue that one of the xy data curve is always behind the others
        #     # rospy.Time() does not solve the issue either
        #     self.v_exist = True
        #     # print((self.v_trans[0] - self.c_x)**2 + (self.v_trans[1] - self.c_y)**2)
        #     self.cart_err_list.append(math.sqrt((self.v_trans[0] - self.c_x)**2 + (self.v_trans[1] - self.c_y)**2))  
        #     self.odom_err_list.append(math.sqrt((self.v_trans[0] - self.o_x)**2 + (self.v_trans[1] - self.o_y)**2))

        #     # print(self.err_list)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.loginfo("Failed to get transform between odom and base_link")
        #     self.v_exist = False
        #     return
        
        # # This gives the transform in a form similiar to a message from a subscriber so we can get the timestamp from the header
        # try:
        #     self.v_tf = self.tf2_buffer.lookup_transform('odom', 'base_link', rospy.Time())
        #     # print(self.v_tf)
        #     self.v_exist = True   

        #     self.v_x = self.v_tf.transform.translation.x
        #     self.v_y = self.v_tf.transform.translation.y 
        #     self.v_x_t_list.append(self.v_x)
        #     self.v_y_t_list.append(self.v_y)
        #     self.v_stamp = self.v_tf.header.stamp

        #     self.cart_err_list.append(math.sqrt((self.v_x - self.c_x)**2 + (self.v_y - self.c_y)**2))  
        #     self.odom_err_list.append(math.sqrt((self.v_x - self.o_x)**2 + (self.v_y - self.o_y)**2))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("Failed to get transform between odom and base_link")
        #     self.v_exist = False
        #     return


    def odom_callback(self, msg):
        self.o_x = msg.pose.pose.position.x
        self.o_y = msg.pose.pose.position.y
        self.o_z = msg.pose.pose.position.z
        self.o_orient_x = msg.pose.pose.orientation.x
        self.o_orient_y = msg.pose.pose.orientation.y
        self.o_orient_z = msg.pose.pose.orientation.z
        self.o_orient_w = msg.pose.pose.orientation.w
        self.o_stamp = msg.header.stamp

        self.o_x_list.append(self.o_x)
        self.o_y_list.append(self.o_y)

    def plot_pose(self, _):
        """
        Plots the robot's XY position in real-time.

        Args:
            _ (Any): Placeholder parameter required by `FuncAnimation`. It is unused in this method.

        This method retrieves the stored position data and plots it on a Matplotlib figure,
        providing a visual representation of the robot's movement in the XY plane.
        """

        x = np.array(self.x_list)
        y = np.array(self.y_list)

        # o_x = np.array(self.o_x_list)
        # o_y = np.array(self.o_y_list)
        
        self.ax_xy.plot(x, y, c='r', marker='.')
        # self.ax_xy.scatter(o_x, o_y, c='g', marker='.')
        lab = ['Pose']
        self.ax_xy.legend(lab, loc="best")
        self.ax_xy.set_xlabel("X")
        self.ax_xy.set_ylabel("Y")
        self.ax_xy.axis("equal")
        self.ax_xy.set_title("The Position of the Robot in XY Plane")
        return self.ax_xy
        
    # def plot_x(self, _):
    #     if self.v_exist:
    #         self.ax_x.scatter((self.v_stamp.secs + self.v_stamp.nsecs*1e-9), self.v_x, c='b', marker='.')
    #     self.ax_x.scatter((self.c_stamp.secs + self.c_stamp.nsecs*1e-9), self.c_x, c='r', marker='.')
    #     self.ax_x.scatter((self.o_stamp.secs + self.o_stamp.nsecs*1e-9), self.o_x, c='g', marker='.')
    #     if self.v_exist:
    #         lab = ['Vicon', 'Cartographer', 'Ekf Odom']
    #     else:
    #         lab = ['Cartographer', 'Ekf Odom']
    #     self.ax_x.legend(lab, loc="best")
    #     self.ax_x.set_xlabel("Time")
    #     self.ax_x.set_ylabel("X")
    #     self.ax_x.set_title("The Position of the Robot in X direction")
    #     return self.ax_x
    
    # def plot_y(self, _):
    #     # else:
    #     #     lab = ['Cartographer', 'Ekf Odom']
    #     self.ax_y.scatter((self.c_stamp.secs + self.c_stamp.nsecs*1e-9), self.c_y, c='r', marker='.')
    #     self.ax_y.scatter((self.o_stamp.secs + self.o_stamp.nsecs*1e-9), self.o_y, c='g', marker='.')
    #     if self.v_exist:
    #         self.ax_y.scatter((self.v_stamp.secs + self.v_stamp.nsecs*1e-9), self.v_y, c='b', marker='.')
    #         lab = ['Cartographer', 'Ekf Odom', 'Vicon']
    #         self.ax_y.legend(lab, loc="best")
    #     self.ax_y.set_xlabel("Time")
    #     self.ax_y.set_ylabel("Y")
    #     self.ax_y.set_title("The Position of the Robot in Y direction")
    #     return self.ax_y


    def _plot(self):
        """Function for initializing and showing matplotlib animation."""
        print("plotting")

        self.ani = animation.FuncAnimation(self.fig_xy, self.plot_pose, interval=10) # frames=100, blit=False)

        # self.ani_x = anim.FuncAnimation(self.fig_x, self.plot_x, interval=10)
        # self.ani_y = anim.FuncAnimation(self.fig_y, self.plot_y, interval=10)
        
        # self.fig_xy.savefig(r'/your/path/to/file.pdf', bbox_inches='tight')

        plt.show(block=True)
        print("here")


def main(args=None):
    rclpy.init(args=args)
    
    ac = PlotPose()
    ac._plot()
    rclpy.spin(ac)
    ac.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()



"""For reference: https://answers.ros.org/question/264767/plotting-real-time-data/#:~:text=Here%20I%20used%20matplotlib%20FuncAnimation%20function.

For ROS2, we need to use multi-threading to do this. Reference: https://github.com/timdodge54/matplotlib_ros_tutorials?tab=readme-ov-file#example-using-ros """