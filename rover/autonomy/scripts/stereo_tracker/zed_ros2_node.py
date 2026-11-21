import glob
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

from stereo_tracker import StereoRig, StereoTracker, annotate


def find_zed_device() -> str:
    """Scan V4L2 devices and return the first /dev/videoX path belonging to a ZED camera."""
    for name_path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        try:
            with open(name_path, "r") as f:
                if "ZED" in f.read().upper():
                    dev_name = os.path.basename(os.path.dirname(name_path))
                    return f"/dev/{dev_name}"
        except OSError:
            continue
    return "/dev/video0"


class ZedStereoTrackerNode(Node):
    def __init__(self):
        super().__init__("zed_stereo_tracker")

        self.declare_parameter("cam", -1)
        self.declare_parameter("calib", "stereo_calib.npz")
        self.declare_parameter("pose_model", 1)
        self.declare_parameter("min_vis", 0.6)
        self.declare_parameter("max_reproj_px", 3.0)
        self.declare_parameter("show_window", True)
        self.declare_parameter("display_scale", 0.6)

        cam_idx = self.get_parameter("cam").value
        calib_path = self.get_parameter("calib").value
        pose_model = self.get_parameter("pose_model").value
        min_vis = self.get_parameter("min_vis").value
        max_reproj = self.get_parameter("max_reproj_px").value
        self.show_window = self.get_parameter("show_window").value
        self.display_scale = self.get_parameter("display_scale").value

        # Resolve calib path relative to this script if not given as absolute
        if not os.path.isabs(calib_path):
            calib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), calib_path)

        dev_path = find_zed_device() if cam_idx < 0 else f"/dev/video{cam_idx}"

        self.dist_pub = self.create_publisher(Float32, "/person_tracker/distance_m", 10)
        self.depth_pub = self.create_publisher(Float32, "/person_tracker/depth_m", 10)
        self.height_pub = self.create_publisher(Float32, "/person_tracker/height_m", 10)
        self.speed_pub = self.create_publisher(Float32, "/person_tracker/speed_mps", 10)
        self.vel_pub = self.create_publisher(Vector3, "/person_tracker/velocity", 10)

        self.rig = StereoRig.from_file(calib_path)
        self.w, self.h = self.rig.size
        self.tracker = StereoTracker(self.rig, pose_model, min_vis, max_reproj)

        self.cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(dev_path)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera device {dev_path}")
            raise RuntimeError("Camera open failed")

        self.get_logger().info(f"ZED ROS 2 Node started (device={dev_path}, size={self.w*2}x{self.h})")
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 2):
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)

        fL = frame[:, :self.w]
        fR = frame[:, self.w:]
        t = time.time()

        info = self.tracker.update(fL, fR, t)

        if info["distance_m"] is not None:
            self.dist_pub.publish(Float32(data=float(info["distance_m"])))
            self.depth_pub.publish(Float32(data=float(info["depth_m"])))

        if info["height_m"] is not None:
            self.height_pub.publish(Float32(data=float(info["height_m"])))

        v = info["velocity"]
        if v is not None:
            self.speed_pub.publish(Float32(data=float(v["speed"])))
            vx, vy, vz = v["v"]
            self.vel_pub.publish(Vector3(x=float(vx), y=float(vy), z=float(vz)))

        if self.show_window:
            self.tracker.draw(fL, fR)
            annotate(fL, info)
            both = np.hstack([fL, fR])
            cv2.imshow(
                "ZED ROS 2 Tracker",
                cv2.resize(both, None, fx=self.display_scale, fy=self.display_scale),
            )
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                rclpy.shutdown()

    def destroy_node(self):
        self.tracker.close()
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedStereoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
