import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from DEM_elevation_map_generator import run_dem_astar
import types, time

class QuickTest(Node):
    def __init__(self):
        super().__init__('dem_test_node')
        self.locations = None
        self.current_position = None
        
        self.create_subscription(Float32MultiArray, '/long_lat_goal_array', self.coord_cb, 10)
        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_cb, 10)

    def coord_cb(self, msg):
        self.locations = {
            "start": (msg.data[0], msg.data[1]),
            "GNSS1": (msg.data[2], msg.data[3]),
        }

    def odom_cb(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

rclpy.init()
node = QuickTest()

# Wait for real data to arrive
print("Waiting for GPS and odom...")
while node.locations is None or node.current_position is None:
    rclpy.spin_once(node, timeout_sec=1.0)

print("Got data, running DEM...")
run_dem_astar(node, "GNSS1")
node.destroy_node()
rclpy.shutdown()
