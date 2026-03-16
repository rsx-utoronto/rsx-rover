#!/usr/bin/env python3
"""
ZED 2i depth viewer + 2D top-down carpet/occupancy map
with global-path / obstacle-avoidance navigation overlay.

NEW in this version
───────────────────
• ROS 2 background thread subscribes to:
    /goal_pose          (geometry_msgs/PoseStamped)  – navigation target
    /zed/zed_node/odom  (nav_msgs/Odometry)          – rover pose
    /rover_nav/state    (std_msgs/String, JSON)       – avoider status
• Carpet-map overlay draws:
    ─ CYAN line    : straight-line planned path (rover → goal in camera frame)
    ─ YELLOW dot   : goal projected into the camera-relative map
    ─ ORANGE rect  : forward danger zone used by the avoider node
    ─ Status panel : nav mode + distance to goal + heading error
• Depth-view HUD shows nav mode, distance, and heading error.
• --goal X Y  CLI arg pre-loads a goal without needing a separate publisher.
• --corridor  half-width of the forward danger zone shown on the map (metres).
• --avoid-dist distance at which the avoider triggers (metres).

Everything else (depth viewer, carpet map, resource logger) is unchanged.

ZED coordinate system (camera frame):
  X  →  right
  Y  ↓  down  (positive Y = below the camera optical centre)
  Z  →  forward (depth)
"""

import sys
import time
import argparse
import json
import math
import threading
import os
import subprocess

import numpy as np
import cv2
import psutil

import pyzed.sl as sl

# ROS 2 is optional – the script runs standalone if rclpy is not available.
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Resource logger
# ─────────────────────────────────────────────────────────────────────────────

def _bytes_to_gib(x: float) -> float:
    return x / (1024 ** 3)


def _get_nvidia_gpu_summary() -> str:
    try:
        cmd = [
            "nvidia-smi",
            "--query-gpu=index,utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True).strip()
        if not out:
            return "GPU: n/a"
        lines = []
        for line in out.splitlines():
            idx, util, mem_used, mem_total = [p.strip() for p in line.split(",")]
            lines.append(f"GPU {idx}: {util}% util, {mem_used}/{mem_total} MiB")
        return " | ".join(lines)
    except Exception:
        return "GPU: n/a"


def start_resource_logger(interval_s: float = 10.0) -> threading.Event:
    stop_event = threading.Event()
    proc = psutil.Process(os.getpid())
    proc.cpu_percent(interval=None)
    psutil.cpu_percent(interval=None)

    def loop():
        while not stop_event.is_set():
            stop_event.wait(interval_s)
            if stop_event.is_set():
                break
            cpu_total = psutil.cpu_percent(interval=None)
            vm = psutil.virtual_memory()
            cpu_proc = proc.cpu_percent(interval=None)
            rss_gib = _bytes_to_gib(proc.memory_info().rss)
            num_threads = proc.num_threads()
            gpu = _get_nvidia_gpu_summary()
            print(
                "\n[RESOURCE] "
                f"System CPU: {cpu_total:5.1f}% | "
                f"RAM: {vm.percent:5.1f}% ({_bytes_to_gib(vm.used):.2f}/{_bytes_to_gib(vm.total):.2f} GiB) | "
                f"Process CPU: {cpu_proc:5.1f}% | "
                f"Process RSS: {rss_gib:.2f} GiB | "
                f"Threads: {num_threads} | "
                f"{gpu}",
                flush=True,
            )

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return stop_event


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 nav-state subscriber (runs in its own thread)
# ─────────────────────────────────────────────────────────────────────────────

class NavState:
    """Thread-safe container for the latest navigation state from ROS 2."""

    def __init__(self):
        self._lock = threading.Lock()
        self.goal_x:    float | None = None
        self.goal_y:    float | None = None
        self.rover_x:   float = 0.0
        self.rover_y:   float = 0.0
        self.rover_yaw: float = 0.0
        self.mode:      str   = "NO GOAL"
        self.dist:      float = 0.0
        self.avoid_dir: float = 0.0   # +1 = turning left, -1 = turning right

    def set_goal(self, x: float, y: float):
        with self._lock:
            self.goal_x = x
            self.goal_y = y

    def set_pose(self, x: float, y: float, yaw: float):
        with self._lock:
            self.rover_x   = x
            self.rover_y   = y
            self.rover_yaw = yaw

    def set_nav(self, mode: str, dist: float, avoid_dir: float):
        with self._lock:
            self.mode      = mode
            self.dist      = dist
            self.avoid_dir = avoid_dir

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "goal_x":    self.goal_x,
                "goal_y":    self.goal_y,
                "rover_x":   self.rover_x,
                "rover_y":   self.rover_y,
                "rover_yaw": self.rover_yaw,
                "mode":      self.mode,
                "dist":      self.dist,
                "avoid_dir": self.avoid_dir,
            }


def _start_ros2_listener(nav: NavState) -> threading.Thread | None:
    """Spin a minimal ROS 2 node in a background daemon thread."""
    if not _ROS2_AVAILABLE:
        print("[OVERLAY] rclpy not found – nav overlay disabled. "
              "Start the ZED ROS 2 wrapper and waypoint_avoider_node separately.")
        return None

    def _spin():
        rclpy.init(args=None)
        node = rclpy.create_node("zed_visualiser_overlay")

        def _goal_cb(msg: PoseStamped):
            nav.set_goal(msg.pose.position.x, msg.pose.position.y)

        def _odom_cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            nav.set_pose(p.x, p.y, yaw)

        def _state_cb(msg: String):
            try:
                d = json.loads(msg.data)
                nav.set_nav(
                    d.get("mode",      "UNKNOWN"),
                    float(d.get("dist", 0.0)),
                    float(d.get("avoid_dir", 0.0)),
                )
            except Exception:
                pass

        node.create_subscription(PoseStamped, "/goal_pose",         _goal_cb,  10)
        node.create_subscription(Odometry,    "/zed/zed_node/odom", _odom_cb,  10)
        node.create_subscription(String,      "/rover_nav/state",   _state_cb, 10)

        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    t = threading.Thread(target=_spin, daemon=True, name="ros2_overlay")
    t.start()
    return t


# ─────────────────────────────────────────────────────────────────────────────
# Nav overlay drawing
# ─────────────────────────────────────────────────────────────────────────────

def _odom_to_map_px(
    cam_x_m: float,
    cam_z_m: float,
    half_width_m: float,
    max_depth_m: float,
    map_px: int,
) -> tuple[int, int]:
    """
    Convert a point in camera frame (X=right, Z=forward) to carpet map pixel.
    Returns (px_col, px_row) clamped to [0, map_px-1].
    """
    px_col = int((cam_x_m + half_width_m) / (2.0 * half_width_m) * map_px)
    px_row = int((1.0 - cam_z_m / max_depth_m) * map_px)
    px_col = max(0, min(map_px - 1, px_col))
    px_row = max(0, min(map_px - 1, px_row))
    return px_col, px_row


def draw_nav_overlay(
    map_img:      np.ndarray,
    snap:         dict,
    half_width_m: float,
    max_depth_m:  float,
    map_px:       int,
    avoid_dist_m: float = 1.0,
    corridor_m:   float = 0.6,
) -> np.ndarray:
    """
    Draw navigation overlays onto the rendered carpet map image (in-place copy).

    Overlays (from back to front so important info stays readable):
      1. Semi-transparent orange danger zone rectangle
      2. Cyan planned-path line (rover origin → goal)
      3. Yellow goal circle (or off-screen arrow if goal is behind rover)
      4. Status text panel (bottom-left)
    """
    img = map_img.copy()

    rover_x   = snap["rover_x"]
    rover_y   = snap["rover_y"]
    yaw       = snap["rover_yaw"]
    goal_x    = snap["goal_x"]
    goal_y    = snap["goal_y"]
    mode      = snap["mode"]
    dist      = snap["dist"]
    avoid_dir = snap["avoid_dir"]

    # ── 1. Danger zone (semi-transparent orange rectangle) ────────────────────
    # Forward corridor: Z in [0, avoid_dist_m], X in [-corridor_m, +corridor_m]
    z_near_col, z_near_row = _odom_to_map_px(-corridor_m, 0.1,          half_width_m, max_depth_m, map_px)
    z_far_col,  z_far_row  = _odom_to_map_px( corridor_m, avoid_dist_m, half_width_m, max_depth_m, map_px)

    overlay = img.copy()
    cv2.rectangle(
        overlay,
        (z_near_col, z_far_row),   # top-left  (far depth, left edge)
        (z_far_col,  z_near_row),  # bot-right (near depth, right edge)
        (0, 140, 255),             # orange BGR
        -1,
    )
    cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)

    # Danger zone border
    cv2.rectangle(
        img,
        (z_near_col, z_far_row),
        (z_far_col,  z_near_row),
        (0, 140, 255),
        1,
    )

    # ── 2. Goal path + marker ─────────────────────────────────────────────────
    if goal_x is not None and goal_y is not None:
        # Transform goal from odom frame → camera frame (Z=forward, X=right)
        dx = goal_x - rover_x
        dy = goal_y - rover_y
        cam_z =  dx * math.cos(yaw) + dy * math.sin(yaw)
        cam_x = -dx * math.sin(yaw) + dy * math.cos(yaw)

        # Rover origin in map pixels (camera = bottom centre)
        rover_px = (map_px // 2, map_px - 4)

        # Goal in map pixels
        goal_px = _odom_to_map_px(cam_x, cam_z, half_width_m, max_depth_m, map_px)

        # Planned path line
        cv2.line(img, rover_px, goal_px, (255, 255, 0), 2, cv2.LINE_AA)   # cyan

        if cam_z > 0:
            # Goal is in front – draw circle
            cv2.circle(img, goal_px, 8, (0, 255, 255), -1)                 # yellow fill
            cv2.circle(img, goal_px, 8, (255, 255, 255), 1)                # white border
            # Distance label next to goal
            cv2.putText(img, f"{dist:.1f}m", (goal_px[0] + 10, goal_px[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            # Goal is behind rover – draw an arrow at the bottom edge
            arrow_x = max(8, min(map_px - 8,
                          int((cam_x + half_width_m) / (2.0 * half_width_m) * map_px)))
            cv2.arrowedLine(img, (arrow_x, map_px - 20), (arrow_x, map_px - 4),
                            (0, 255, 255), 2, cv2.LINE_AA, tipLength=0.5)
            cv2.putText(img, "BEHIND", (arrow_x - 18, map_px - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1, cv2.LINE_AA)

    # ── 3. Avoid-turn indicator ───────────────────────────────────────────────
    if mode == "AVOIDING":
        arrow_col = (0, 140, 255)  # orange
        cx = map_px // 2
        cy = map_px - 30
        if avoid_dir > 0:    # turning left
            cv2.arrowedLine(img, (cx, cy), (cx - 25, cy), arrow_col, 2,
                            cv2.LINE_AA, tipLength=0.4)
        elif avoid_dir < 0:  # turning right
            cv2.arrowedLine(img, (cx, cy), (cx + 25, cy), arrow_col, 2,
                            cv2.LINE_AA, tipLength=0.4)

    # ── 4. Status panel ───────────────────────────────────────────────────────
    mode_colour = {
        "NAVIGATING":   (0, 220, 80),    # green
        "AVOIDING":     (0, 140, 255),   # orange
        "GOAL REACHED": (255, 220, 0),   # cyan-ish
        "NO GOAL":      (160, 160, 160), # grey
    }.get(mode, (200, 200, 200))

    cv2.putText(img, mode, (6, map_px - 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, mode_colour, 1, cv2.LINE_AA)
    if goal_x is not None:
        cv2.putText(img, f"goal ({goal_x:.1f}, {goal_y:.1f})", (6, map_px - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1, cv2.LINE_AA)

    return img


def draw_hud(
    vis_color: np.ndarray,
    snap:      dict,
    y_offset:  int = 90,
) -> np.ndarray:
    """
    Overlay navigation status on the depth image (top-left corner).
    Placed below the existing FPS counter (which is at y=60).
    """
    mode      = snap["mode"]
    dist      = snap["dist"]
    goal_x    = snap["goal_x"]
    goal_y    = snap["goal_y"]
    rover_x   = snap["rover_x"]
    rover_y   = snap["rover_y"]
    yaw       = snap["rover_yaw"]

    mode_colour = {
        "NAVIGATING":   (80, 220, 80),
        "AVOIDING":     (0, 140, 255),
        "GOAL REACHED": (0, 255, 220),
        "NO GOAL":      (180, 180, 180),
    }.get(mode, (200, 200, 200))

    lines = [f"NAV: {mode}"]
    if goal_x is not None:
        lines.append(f"Goal ({goal_x:.2f}, {goal_y:.2f})  dist {dist:.2f}m")
        # Heading error
        dx = goal_x - rover_x
        dy = goal_y - rover_y
        angle_to_goal = math.atan2(dy, dx)
        err_rad = angle_to_goal - yaw
        while err_rad >  math.pi: err_rad -= 2 * math.pi
        while err_rad < -math.pi: err_rad += 2 * math.pi
        lines.append(f"Heading err {math.degrees(err_rad):.1f}°  "
                     f"Rover ({rover_x:.2f}, {rover_y:.2f})")

    for i, line in enumerate(lines):
        colour = mode_colour if i == 0 else (220, 220, 220)
        cv2.putText(vis_color, line, (10, y_offset + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, colour, 2, cv2.LINE_AA)

    return vis_color


# ─────────────────────────────────────────────────────────────────────────────
# 2D carpet / occupancy map  (unchanged logic, render() now returns base only)
# ─────────────────────────────────────────────────────────────────────────────

class CarpetMap:
    """
    Lightweight top-down occupancy grid built from a strided XYZ point cloud.

    ZED coordinate frame (camera origin):
      X → right   Y ↓ down (positive = toward floor)   Z → forward
    """

    def __init__(
        self,
        max_depth_m:  float = 8.0,
        half_width_m: float = 4.0,
        cell_m:       float = 0.10,
        y_min:        float = 0.0,
        y_max:        float = 0.8,
        min_depth_m:  float = 0.5,
        stride:       int   = 4,
        ema_alpha:    float = 0.4,
        map_px:       int   = 500,
    ):
        self.max_depth_m  = max_depth_m
        self.half_width_m = half_width_m
        self.cell_m       = cell_m
        self.y_min        = y_min
        self.y_max        = y_max
        self.min_depth_m  = min_depth_m
        self.stride       = stride
        self.ema_alpha    = ema_alpha
        self.map_px       = map_px

        self.cols = int(round(2 * half_width_m / cell_m))
        self.rows = int(round(max_depth_m / cell_m))

        self._occ      = np.zeros((self.rows, self.cols), dtype=np.float32)
        self._template = self._build_template()

    def update(self, xyz: np.ndarray) -> None:
        s   = self.stride
        sub = xyz[::s, ::s]
        X   = sub[:, :, 0].ravel()
        Y   = sub[:, :, 1].ravel()
        Z   = sub[:, :, 2].ravel()

        mask = (
            np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
            & (Z >= self.min_depth_m)  & (Z <= self.max_depth_m)
            & (np.abs(X) <= self.half_width_m)
            & (Y >= self.y_min)        & (Y < self.y_max)
        )
        Xo, Zo = X[mask], Z[mask]

        hit = np.zeros((self.rows, self.cols), dtype=np.float32)
        if Xo.size > 0:
            col = np.clip(
                np.floor((Xo + self.half_width_m) / self.cell_m).astype(np.int32),
                0, self.cols - 1,
            )
            row = np.clip(
                np.floor(Zo / self.cell_m).astype(np.int32),
                0, self.rows - 1,
            )
            hit[row, col] = 1.0

        self._occ = self.ema_alpha * hit + (1.0 - self.ema_alpha) * self._occ

    def render(self) -> np.ndarray:
        """Returns a (map_px, map_px, 3) BGR base image (obstacles + grid only)."""
        obstacle = self._occ >= 0.5
        img = np.zeros((self.rows, self.cols, 3), dtype=np.uint8)
        img[obstacle] = (0, 0, 220)   # red

        img = np.flipud(img)
        img = cv2.resize(img, (self.map_px, self.map_px), interpolation=cv2.INTER_NEAREST)
        cv2.add(img, self._template, dst=img)
        return img

    def _build_template(self) -> np.ndarray:
        px     = self.map_px
        t      = np.zeros((px, px, 3), dtype=np.uint8)
        scx    = px / (2 * self.half_width_m)
        scz    = px / self.max_depth_m
        gc     = (45, 45, 45)

        for x in np.arange(-self.half_width_m, self.half_width_m + 0.5, 1.0):
            cx = int((x + self.half_width_m) * scx)
            cv2.line(t, (cx, 0), (cx, px - 1), gc, 1)

        for z in range(0, int(self.max_depth_m) + 1):
            ry = int(px - 1 - z * scz)
            if 0 <= ry < px:
                cv2.line(t, (0, ry), (px - 1, ry), gc, 1)
                if z > 0:
                    cv2.putText(t, f"{z}m", (4, max(ry - 3, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                                (100, 100, 100), 1, cv2.LINE_AA)

        cx, cy = px // 2, px - 4
        pts = np.array([[cx, cy - 12], [cx - 8, cy + 2], [cx + 8, cy + 2]], np.int32)
        cv2.fillPoly(t, [pts], (0, 210, 80))

        cv2.putText(t, "TOP-DOWN MAP", (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(t, "red=obstacle  cyan=path  yellow=goal  orange=danger zone",
                    (6, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (120, 120, 120), 1,
                    cv2.LINE_AA)
        return t


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ZED 2i depth viewer + 2D top-down carpet map + nav overlay."
    )
    parser.add_argument("--max-depth",   type=float, default=10.0)
    parser.add_argument("--fps",         type=int,   default=15)
    parser.add_argument("--res",         type=str,   default="HD2K",
                        choices=["VGA", "HD720", "HD1080", "HD2K"])
    parser.add_argument("--debug",       action="store_true")

    # Carpet map
    parser.add_argument("--y-min",      type=float, default=0.0)
    parser.add_argument("--y-max",      type=float, default=0.8)
    parser.add_argument("--min-depth",  type=float, default=0.5)
    parser.add_argument("--stride",     type=int,   default=4)
    parser.add_argument("--cell",       type=float, default=0.10)
    parser.add_argument("--ema",        type=float, default=0.4)
    parser.add_argument("--map-width",  type=float, default=4.0)
    parser.add_argument("--map-px",     type=int,   default=500)

    # Nav overlay
    parser.add_argument("--goal",       type=float, nargs=2, metavar=("X", "Y"),
                        default=None,
                        help="Pre-load a goal in odom metres, e.g. --goal 5.0 2.0")
    parser.add_argument("--avoid-dist", type=float, default=1.0,
                        help="Avoider trigger distance shown as danger zone (m).")
    parser.add_argument("--corridor",   type=float, default=0.6,
                        help="Half-width of the forward danger zone corridor (m).")
    args = parser.parse_args()

    # ── Nav state ─────────────────────────────────────────────────────────────
    nav = NavState()
    if args.goal is not None:
        nav.set_goal(args.goal[0], args.goal[1])
    _start_ros2_listener(nav)

    # ── ZED ───────────────────────────────────────────────────────────────────
    res_map = {
        "VGA":    sl.RESOLUTION.VGA,
        "HD720":  sl.RESOLUTION.HD720,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD2K":   sl.RESOLUTION.HD2K,
    }

    zed  = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution        = res_map[args.res]
    init.camera_fps               = args.fps
    init.coordinate_units         = sl.UNIT.METER
    init.depth_maximum_distance   = args.max_depth

    for mode_name in ("NEURAL_PLUS", "NEURAL", "ULTRA", "QUALITY"):
        if hasattr(sl.DEPTH_MODE, mode_name):
            init.depth_mode = getattr(sl.DEPTH_MODE, mode_name)
            break

    init.depth_stabilization = 50

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED: {repr(status)}")
        sys.exit(1)

    stop_logger = start_resource_logger(interval_s=10.0)

    cam_info = zed.get_camera_information()
    h = cam_info.camera_configuration.resolution.height
    w = cam_info.camera_configuration.resolution.width
    print(f"Opened ZED: {cam_info.camera_model} | {w}x{h} | "
          f"{cam_info.camera_configuration.fps} FPS")

    # Lock exposure/gain
    try:
        if hasattr(sl, "VIDEO_SETTINGS") and hasattr(zed, "set_camera_settings"):
            if hasattr(sl.VIDEO_SETTINGS, "AEC_AGC"):
                zed.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
            if hasattr(sl.VIDEO_SETTINGS, "EXPOSURE"):
                zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
            if hasattr(sl.VIDEO_SETTINGS, "GAIN"):
                zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 50)
    except Exception as e:
        print(f"Camera settings lock skipped: {e}")

    # ROI: ignore top 40 %
    y0 = int(0.40 * h)
    roi_mask_np = np.zeros((h, w), dtype=np.uint8)
    roi_mask_np[y0:, :] = 255

    roi_mask_sl   = sl.Mat(w, h, sl.MAT_TYPE.U8_C1)
    roi_mask_view = roi_mask_sl.get_data()
    if roi_mask_view.ndim == 3 and roi_mask_view.shape[2] == 1:
        roi_mask_view = roi_mask_view[:, :, 0]
    if roi_mask_view.shape[:2] == roi_mask_np.shape:
        np.copyto(roi_mask_view, roi_mask_np)
    elif roi_mask_view.shape[:2] == roi_mask_np.T.shape:
        np.copyto(roi_mask_view, roi_mask_np.T)
    else:
        raise RuntimeError(
            f"ROI mask shape mismatch: view={roi_mask_view.shape}, np={roi_mask_np.shape}")

    if hasattr(zed, "setRegionOfInterest"):
        zed.setRegionOfInterest(roi_mask_sl)
    elif hasattr(zed, "set_region_of_interest"):
        zed.set_region_of_interest(roi_mask_sl)

    runtime = sl.RuntimeParameters()
    if hasattr(runtime, "confidence_threshold"):
        runtime.confidence_threshold = 50
    if hasattr(runtime, "texture_confidence_threshold"):
        runtime.texture_confidence_threshold = 100
    if hasattr(sl, "SENSING_MODE"):
        runtime.sensing_mode = (
            getattr(sl.SENSING_MODE, "STANDARD", None)
            or getattr(sl.SENSING_MODE, "FILL", None)
        )

    depth_mat = sl.Mat()
    xyz_mat   = sl.Mat()

    # ── Windows ───────────────────────────────────────────────────────────────
    win_depth = "ZED Depth (metres) — ESC to quit"
    win_map   = "Carpet Map + Nav Overlay"
    cv2.namedWindow(win_depth, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_map,   cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_map, args.map_px, args.map_px)

    mouse = {"x": 0, "y": 0, "valid": False}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            mouse["x"], mouse["y"] = x, y
            mouse["valid"] = True

    cv2.setMouseCallback(win_depth, on_mouse)

    carpet = CarpetMap(
        max_depth_m  = args.max_depth,
        half_width_m = args.map_width,
        cell_m       = args.cell,
        y_min        = args.y_min,
        y_max        = args.y_max,
        min_depth_m  = args.min_depth,
        stride       = args.stride,
        ema_alpha    = args.ema,
        map_px       = args.map_px,
    )

    last_t      = time.time()
    frame_count = 0
    shown_fps   = 0.0
    prev_depth  = None
    alpha       = 0.8

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            # ── Depth ─────────────────────────────────────────────────────────
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth_full = np.squeeze(depth_mat.get_data())
            depth      = depth_full[y0:, :]

            # ── Point cloud ───────────────────────────────────────────────────
            zed.retrieve_measure(xyz_mat, sl.MEASURE.XYZ)
            xyz_full = xyz_mat.get_data()
            xyz      = xyz_full[y0:, :, :]

            # ── Temporal smoothing ────────────────────────────────────────────
            d   = depth.copy()
            bad = ~np.isfinite(d)
            d[bad] = args.max_depth
            d = np.clip(d, 0.0, args.max_depth)

            if prev_depth is None:
                prev_depth = d.copy()
            else:
                valid  = np.isfinite(depth) & (depth > 0)
                diff   = np.abs(d - prev_depth)
                motion = float(np.mean(diff[valid])) if np.any(valid) else 0.0
                if motion > 0.08:
                    prev_depth = d.copy()
                else:
                    prev_depth[valid]  = (1 - alpha) * prev_depth[valid] + alpha * d[valid]
                    prev_depth[~valid] = args.max_depth
                d = prev_depth

            # ── Depth visualisation ───────────────────────────────────────────
            vis       = (1.0 - (d / args.max_depth)) * 255.0
            vis_color = cv2.applyColorMap(vis.astype(np.uint8), cv2.COLORMAP_TURBO)

            if mouse["valid"]:
                mx, my = mouse["x"], mouse["y"]
                dh, dw = d.shape[:2]
                if 0 <= mx < dw and 0 <= my < dh:
                    my_full = my + y0
                    depth_m = float(depth_full[my_full, mx]) \
                              if np.isfinite(depth_full[my_full, mx]) else float("nan")
                    label   = f"Depth: {depth_m:.3f} m"
                    p       = xyz_full[my_full, mx]
                    X, Y, Z = float(p[0]), float(p[1]), float(p[2])
                    if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                        label += f" | XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f}) m"
                    else:
                        label += " | XYZ: (nan)"
                    cv2.circle(vis_color, (mx, my), 4, (255, 255, 255), -1)
                    cv2.putText(vis_color, label, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            frame_count += 1
            now = time.time()
            if now - last_t >= 1.0:
                shown_fps   = frame_count / (now - last_t)
                frame_count = 0
                last_t      = now
                if args.debug:
                    print(f"Viewer FPS: {shown_fps:.1f}")

            cv2.putText(vis_color, f"FPS: {shown_fps:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            # ── Nav HUD on depth image ─────────────────────────────────────────
            snap = nav.snapshot()
            draw_hud(vis_color, snap, y_offset=90)

            # ── Carpet map + nav overlay ───────────────────────────────────────
            carpet.update(xyz)
            map_img = carpet.render()
            map_img = draw_nav_overlay(
                map_img,
                snap,
                half_width_m = args.map_width,
                max_depth_m  = args.max_depth,
                map_px       = args.map_px,
                avoid_dist_m = args.avoid_dist,
                corridor_m   = args.corridor,
            )

            # ── Show ──────────────────────────────────────────────────────────
            cv2.imshow(win_depth, vis_color)
            cv2.imshow(win_map,   map_img)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:   # ESC
                break

    finally:
        try:
            stop_logger.set()
        except Exception:
            pass
        cv2.destroyAllWindows()
        zed.close()


if __name__ == "__main__":
    main()
