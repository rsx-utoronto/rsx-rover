#!/usr/bin/env python3
"""
Gives goals to move_base
"""
import rclpy
from rclpy.node import Node
#  import actionlib Ros 1 only
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal Ros 1 only
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class MoveBaseGoalSet(Node):
    def __init__(self):
        super().__init__('move_base_goal_set')
        # self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()

    def send_goal(self, pos_x = 0, pos_y = 0, pos_z = 0, orient_x = 0, orient_y = 0, orient_z = 0, orient_w = 1):
        rospy.loginfo("Sending goal")

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = pos_x
        self.goal.target_pose.pose.position.y = pos_y
        self.goal.target_pose.pose.position.z = pos_z
        self.goal.target_pose.pose.orientation.x = orient_x
        self.goal.target_pose.pose.orientation.y = orient_y
        self.goal.target_pose.pose.orientation.z = orient_z
        self.goal.target_pose.pose.orientation.w = orient_w


        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            # rospy.logerr("Action server not available!")
            self.get_logger().error("Action server not available!")
            rclpy.shutdown("Action server not available!")
            
        else:
            self.client.get_result()
            # rospy.loginfo("Goal execution done!")
            self.get_logger().info("Goal execution done!")
            
if __name__ == '__main__':
    rclpy.init(args=None)
    move_base_goal= None
    try:
        move_base_goal = MoveBaseGoalSet()
        move_base_goal.send_goal(1, 1)
        rclpy.spin_once(move_base_goal)  # or spin(), depending on how you implemented goal feedback
    except KeyboardInterrupt:
        pass
    finally:
        if move_base_goal is not None:
           
            move_base_goal.destroy_node()
  
        rclpy.shutdown()