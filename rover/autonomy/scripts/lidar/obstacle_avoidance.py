#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist

'''
Reactive obstacle avoidance node using Ouster LiDAR
Listens to /ouster/points PointCloud2 topic
Publishes velocity commands to /cmd_vel topic
Implements simple reactive behavior:
- Continuously scans point cloud for obstacles in front sector
- If obstacle detected within threshold distance, stops forward motion and turns away
- Resumes forward motion when path is clear

Parameters (can be adjusted as needed):
- cloud_topic: Topic to subscribe for point clouds (default: /ouster/points)
- ground_z_threshold: Minimum z value to consider point as obstacle (default: 0.15 m)
- cmd_topic: Topic to publish velocity commands (default: /cmd_vel)
- forward_speed: Forward linear speed when path is clear (default: 0.3 m/s)
- turn_speed: Angular speed when turning to avoid obstacle (default: 0.6 rad/s)
- avoid_distance: Distance threshold to consider obstacle (default: 1.0 m)
- sector_half_angle: Half-angle of front sector to monitor (default: 0.52 rad ~30 deg)
- lateral_threshold: Lateral distance threshold for obstacle detection (default: 1.0 m)
- turn_angle: Angle to turn when avoiding obstacle (default: 0.6 rad)
- publish_rate: Rate to publish /cmd_vel commands (default: 10.0 Hz)
'''

class ReactiveAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_avoidance')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Declare parameters with defaults. These can be overridden via ros2 param or launch files.
        # cloud_topic: Topic to subscribe for PointCloud2 (raw or preprocessed)
        # ground_z_threshold: Points with z < this are ignored (considered ground)
        # cmd_topic: Topic to publish velocity commands (Twist)
        # forward_speed: Linear speed during DRIVE (m/s)
        # turn_speed: Angular speed during TURN (rad/s)
        # avoid_distance: Distance threshold to consider an obstacle close (m)
        # sector_half_angle: Half-angle of forward detection cone (radians)
        # lateral_threshold: Max lateral distance to consider (meters)
        # turn_angle: Desired angular change (radians) applied open-loop by time
        # publish_rate: How often /cmd_vel is published (Hz)
        self.declare_parameter('cloud_topic', '/ouster/points')
        self.declare_parameter('ground_z_threshold', 0.15)  # m, ignore points below this z (ground)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.3)       # m/s
        self.declare_parameter('turn_speed', 0.6)          # rad/s
        self.declare_parameter('avoid_distance', 1.0)      # m, detect obstacle within this distance
        self.declare_parameter('sector_half_angle', 0.52)  # rad (~30 deg)
        self.declare_parameter('lateral_threshold', 1.0)   # m lateral window
        self.declare_parameter('turn_angle', 0.6)          # rad to rotate when avoiding
        self.declare_parameter('publish_rate', 10.0)       # Hz

        # Retrieve parameter values (use safer getters to get concrete types)
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.ground_z_threshold = self.get_parameter('ground_z_threshold').get_parameter_value().double_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.avoid_distance = self.get_parameter('avoid_distance').get_parameter_value().double_value
        self.sector_half_angle = self.get_parameter('sector_half_angle').get_parameter_value().double_value
        self.lateral_threshold = self.get_parameter('lateral_threshold').get_parameter_value().double_value
        self.turn_angle = self.get_parameter('turn_angle').get_parameter_value().double_value
        self.pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Create subscriber to point cloud topic. The callback is cloud_cb.
        # Using the configured QoS which balances throughput vs reliability.
        self.subscription = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_cb, qos
        )

        # Publisher for velocity commands. Downstream drive controller should listen to this.
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos)

        # Internal state machine:
        # - 'DRIVE': move forward at forward_speed (unless an immediate obstacle is very close)
        # - 'TURN': rotate in-place (no forward motion) for a precomputed duration
        self.state = 'DRIVE'   # DRIVE or TURN
        self.turn_direction = 1  # +1 left, -1 right; chosen by obstacle position
        self.turn_end_time = 0.0 # when current turn should end
        self.last_cloud_time = 0.0 # timestamp of last received cloud
        
        # Create a timer to publish /cmd_vel at a fixed rate (publish_rate).
        period = 1.0 / max(1.0, self.pub_rate) # avoid dividing by zero or too-high freq
        self.timer = self.create_timer(period, self.control_loop)

        # Informative log about startup and which cloud topic is used.
        self.get_logger().info(f'ReactiveAvoidance node started, listening to {cloud_topic}')

    def cloud_cb(self, cloud_msg: PointCloud2):
        """
        Callback for incoming PointCloud2 messages.
        This processes the cloud in an online, single-pass way to detect obstacles
        in a forward sector. It does NOT build a map — only counts and distances.
        """
        # Read points iterator for x,y,z fields; skip_nans=True drops points with NaNs.
        pts_iter = point_cloud2.read_points(cloud_msg, field_names=('x','y','z'), skip_nans=True)
        
        # Counters for detections in the forward sector
        front_count = 0   # total number of points inside the forward detection cone & range
        left_count = 0    # subset of front_count with y >= 0 (left side)
        right_count = 0   # subset with y < 0 (right side)
        min_distance = float('inf')  # track the closest point seen (for emergency stop decision)

        # Iterate through points once; this is efficient for Python-level processing of PointCloud2.
        for p in pts_iter:
            try:
                x = float(p[0]); y = float(p[1]); z = float(p[2])
            except Exception:
                continue # If the point cannot be parsed, skip it (robustness).

            # ignore points below the ground threshold (reduce false positives from ground)
            # If your lidar frame is tilted or terrain varies, tune this parameter.
            if z < self.ground_z_threshold:
                continue

            # Consider only points in front of the robot: here +X is assumed to be forward.
            # If your sensor frame uses a different convention, adjust this test.
            if x <= 0.0:
                continue

            # Euclidean distance in the horizontal plane (x,y)
            dist = math.hypot(x, y)
            if dist < min_distance:
                min_distance = dist

            # Only consider points that are within the avoid_distance and lateral window
            if dist <= self.avoid_distance and abs(y) <= self.lateral_threshold:
                # Angle of point relative to forward (+X) axis
                angle = math.atan2(y, x)
                # If angle lies within the forward sector half-angle, count it
                if abs(angle) <= self.sector_half_angle:
                    front_count += 1
                    # Increment left or right counters for choosing turn direction
                    if y >= 0:
                        left_count += 1
                    else:
                        right_count += 1

        # Save detection results for the control loop (timestamp in wall time).
        self.last_cloud_time = time.time()
        self.latest_detection = {
            'front_count': front_count,
            'left_count': left_count,
            'right_count': right_count,
            'min_distance': min_distance
        }

        # If obstacle detected in front, decide turn direction and start turning
        if front_count > 0:
            # Choose turn direction to the side with fewer close points.
            # Ties prefer left (turn_direction = +1).
            if left_count <= right_count:
                self.turn_direction = 1
            else:
                self.turn_direction = -1

            # Compute turn duration needed to roughly achieve turn_angle using open-loop timing:
            # duration = angle / angular_speed. Use a tiny epsilon to avoid division by zero.
            turn_duration = abs(self.turn_angle) / max(1e-6, abs(self.turn_speed))
            self.turn_end_time = time.time() + turn_duration

            # Enter TURN state; control_loop will apply angular velocity and stop forward motion.
            self.state = 'TURN'
            self.get_logger().info(f'Obstacle detected (front_count={front_count}, L={left_count}, R={right_count}), turning {"left" if self.turn_direction>0 else "right"} for {turn_duration:.2f}s')

    def control_loop(self):
        """
        Timer-driven control loop that publishes Twist messages at a fixed rate.
        It consults the internal state machine (DRIVE or TURN) and latest detection info.
        """
        now = time.time()
        twist = Twist() # default zero velocities
        
        # If we are turning, continue until turn_end_time
        if self.state == 'TURN':
            # If point cloud data has not been updated recently, be conservative and stop.
            # This avoids moving blindly if sensor input stops.
            if now - self.last_cloud_time > 1.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warning('No recent cloud data; stopping')
            else:
                # While turning, do not move forward; apply angular velocity in chosen direction.
                twist.linear.x = 0.0
                twist.angular.z = float(self.turn_direction) * float(self.turn_speed)

            # Exit TURN state once the precomputed time has elapsed.
            if now >= self.turn_end_time:
                self.state = 'DRIVE'
                self.get_logger().info('Turn complete, resuming drive')

        else:  # DRIVE state: normally move forward
            # Safety stop: if the most recent detection reported a point very close to the robot,
            # stop instead of moving forward. This threshold (0.5 m) is conservative; tune as needed.
            if hasattr(self, 'latest_detection') and self.latest_detection.get('min_distance', float('inf')) < 0.5:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Very close obstacle, stopping briefly')
            else:
                # Normal forward command: move straight at configured forward_speed.
                twist.linear.x = float(self.forward_speed)
                twist.angular.z = 0.0

        # Publish the command. Downstream controllers should enforce limits and safety.
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down reactive_avoidance')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()