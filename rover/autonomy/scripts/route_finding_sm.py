#!/usr/bin/env python3
"""
Route-finding state machine

State Machine Diagram: https://lucid.app/lucidspark/de548cc5-ef6a-402f-8165-6ba17ecb4b84/edit?viewport_loc=-1796%2C-218%2C5637%2C3109%2C0_0&invitationId=inv_bac4b471-60e8-4381-884b-c70ed722c39a

States (flowchart box -> state):
  Initialize Autonomous Navigation      INIT                reads GPS from the GUI, converts to x/y,
                                                            moves /pose's origin to Start
  At Mission Start Location? + Red LED  START_GATE          checks /pose is within START_TOLERANCE_M
                                                            of Start; if not, blue LED and back to INIT
                                                            while the operator teleoperates there
  Location Selection                    LOCATION_SELECTION  picks the next target on the shortest
                                                            remaining route from where the rover is
  Rover Cruise + GNSS[i]/AR Tag[i]      CRUISE              turns on tag detection for the target
                                                            (AR1/AR2 on "state") and drives there
                                                            with sm_straight_line.py
  Search/Detect Tag + Display on GUI    CHECK_TAG           watches aruco_found from ar_detection_node.py
  Move Towards Tag + Stop within 1 m    APPROACH_TAG        homes on the tag's bounding box
                                                            (aruco_node/bbox), driving on /drive
  Flash Green LED, Report Arrival       SIGNAL_SUCCESS      green LED + GUI message
  Approval + Programming/adjustments    AWAIT_APPROVAL      flashes green until Next Task (/next_state)
                                                            or APPROVAL_TIMEOUT_S, then goes on
  (abort, any time)                     ABORT               GUI abort button (/auto_abort_check):
                                                            stop, blue LED, operator drives back

- Coordinate frame: targets are metres from Start (x = east, y = north). INIT moves
  gps_to_pose.py's origin to Start (/set_origin) so /pose uses the same frame. Turn this off
  with SET_POSE_ORIGIN_TO_START.
- Target order: the shortest total straight-line route from the rover's current position,
  recomputed every time LOCATION_SELECTION runs.
- Homing: APPROACH_TAG drives the rover itself, because no other node homes on a tag on
  request. It uses the same rules as the Aimer in aruco_homing_improve.py / sm_grid_search.py:
  turn until the tag is near the middle of the image, drive until it looks big enough, then a
  short final push. The Aimer is copied here rather than imported because
  aruco_homing_improve.py loads its config from a fixed /home/rsx path when imported.
- Out of radio range: AWAIT_APPROVAL waits up to APPROVAL_TIMEOUT_S for Next Task, then
  carries on by itself (target 2 is behind the hill). Set it to None to always wait.
- Abort: the GUI abort button also stops sm_straight_line.py, which then won't drive again
  until it restarts. So ABORT stops the mission, turns the LED blue and the operator
  teleoperates back the way the rover came.
"""

import itertools
import math
import os
import queue
import signal
import threading
import time

import rclpy
import smach
import yaml
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String

from rover.msg import MissionState
from rover_qos import STATE_QOS
import gps_conversion_functions as gps

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

# Target names
TARGET_NAMES = ["POST1", "POST2"]

# What to publish on the "state" topic while heading to each target. ar_detection_node.py
# only looks for tags while "state" is AR1 or AR2 (the GNSS[i] / AR Tag[i] boxes).
TARGET_DETECTOR_STATES = {"POST1": "AR1", "POST2": "AR2"}

# Move gps_to_pose.py's origin to Start so /pose is in the same frame as the targets.
# Without this, /pose starts wherever the rover was when gps_to_pose.py launched.
SET_POSE_ORIGIN_TO_START = True

# How close to Start (meters) the rover must be to begin. 
START_TOLERANCE_M = 3.0

# How long START_GATE waits for a fresh /pose before saying the rover isn't at the start.
POSE_TIMEOUT_S = 5.0

# While not at the start, how long START_GATE waits before checking again (via INIT).
START_RECHECK_S = 2.0

# How long CRUISE waits for sm_straight_line.py to answer before giving up on a drive.
CRUISE_TIMEOUT_S = 600

# After arriving, how long CHECK_TAG looks for the tag before moving on.
CHECK_TAG_WINDOW_S = 5.0

# Longest APPROACH_TAG may home on a tag before treating it as lost.
APPROACH_TIMEOUT_S = 60.0

# Homing numbers, the same as sm_grid_search.py's. Speeds come from sm_config.yaml.
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sm_config.yaml")) as _f:
    _sm_config = yaml.safe_load(_f)
HOMING_LIN_VEL = float(_sm_config.get("Ar_homing_lin_vel", 1.8))
HOMING_ANG_VEL = float(_sm_config.get("Ar_homing_ang_vel", 0.5))
# Tag area (pixels^2) that counts as close enough, per camera
HOMING_MIN_TAG_AREA = 2500.0 if _sm_config.get("realsense_detection") else 750.0
HOMING_FRAME_WIDTH = 640           # camera image width the bounding boxes are measured in
HOMING_CENTRE_TOLERANCE_PX = 100   # tag this close to the middle counts as straight ahead
HOMING_FINAL_PUSH_S = 0.7          # once close enough, drive straight this long more
HOMING_LOST_AFTER_S = 2.0          # no bounding box for this long = tag lost

# How many times the rover may lose a tag while approaching before giving up on that target.
MAX_APPROACH_ATTEMPTS = 3

# How long AWAIT_APPROVAL waits for Next Task before continuing on its own (None = forever).
APPROVAL_TIMEOUT_S = 120.0

# Green LED flash half-period while waiting for approval (rule 1.e.ii: flashing green).
FLASH_PERIOD_S = 0.5


# ---------------------------------------------------------------------------
# ROS connection
# ---------------------------------------------------------------------------

class RoverNode(Node):
    """All publishers and subscribers the state machine uses."""

    def __init__(self):
        super().__init__("route_finding_sm")
        self.mission_pub = self.create_publisher(MissionState, "mission_state", 10)
        self.led_pub = self.create_publisher(String, "led_light", 10)
        self.gui_pub = self.create_publisher(String, "gui_status", STATE_QOS)
        # ar_detection_node.py only runs detection while "state" is AR1/AR2
        self.state_pub = self.create_publisher(String, "state", STATE_QOS)
        # gps_to_pose.py: position.x = latitude, position.y = longitude of the new origin
        self.origin_pub = self.create_publisher(Pose, "/set_origin", 10)
        # Drive commands while homing on a tag (sm_straight_line.py drives during CRUISE)
        self.drive_pub = self.create_publisher(Twist, "/drive", 10)

        self.create_subscription(MissionState, "mission_state", self._feedback_callback, 10)
        self.create_subscription(PoseStamped, "/pose", self._pose_callback, 10)
        # ar_detection_node.py publishes True on aruco_found whenever it sees a tag
        self.create_subscription(Bool, "aruco_found", self._aruco_callback, 10)
        # ...and the tag's bounding box: [tl_x, tl_y, tr_x, tr_y, bl_x, bl_y, br_x, br_y]
        self.create_subscription(Float64MultiArray, "aruco_node/bbox", self._bbox_callback, 10)
        self.create_subscription(Float32MultiArray, "/long_lat_goal_array", self._gps_callback, 10)
        # GUI buttons: Next Task and Abort
        self.create_subscription(Bool, "/next_state", self._next_callback, 5)
        self.create_subscription(Bool, "/auto_abort_check", self._abort_callback, 5)

        self.feedback = queue.Queue()
        self.pose = None             # latest PoseStamped from /pose, None until one arrives
        self.pose_time = 0.0         # time.time() when it arrived
        self.last_tag_seen = 0.0     # time.time() when a tag was last seen, 0 = never
        self.bbox = None             # latest tag bounding box (8 numbers), None until one arrives
        self.bbox_time = 0.0         # time.time() when it arrived
        self.gps_values = None       # latest list of numbers from the GUI, None until one arrives
        self.last_next_press = 0.0   # time.time() of the last Next Task press, 0 = never
        self.aborted = False         # set once by the GUI abort button, never cleared

    def _feedback_callback(self, msg):
        self.feedback.put(msg.state)

    def _pose_callback(self, msg):
        self.pose = msg
        self.pose_time = time.time()

    def _aruco_callback(self, msg):
        if msg.data:
            self.last_tag_seen = time.time()

    def _bbox_callback(self, msg):
        if len(msg.data) >= 8:
            self.bbox = list(msg.data[:8])
            self.bbox_time = time.time()

    def _gps_callback(self, msg):
        self.gps_values = list(msg.data)

    def _next_callback(self, msg):
        if msg.data:
            self.last_next_press = time.time()

    def _abort_callback(self, msg):
        if msg.data:
            self.aborted = True

    # helpers
    # Publishing helpers skip publishing once ROS has shut down (e.g. Ctrl+C): publishing then
    # raises an error that would crash the state machine on its way out.
    def set_led(self, mode):
        """mode: "auto" (red), "manual" (blue), "mission done" (green), "off". See led_light.py."""
        if rclpy.ok():
            self.led_pub.publish(String(data=mode))

    def gui(self, text):
        """Status line for the GUI, and printed locally."""
        print(text)
        if rclpy.ok():
            self.gui_pub.publish(String(data=text))

    def drive(self, linear=0.0, angular=0.0):
        """Drive command (m/s forward, rad/s counter-clockwise). drive() alone stops."""
        if rclpy.ok():
            twist = Twist()
            twist.linear.x = float(linear)
            twist.angular.z = float(angular)
            self.drive_pub.publish(twist)

    def set_detector_state(self, state):
        """Tell ar_detection_node.py which tag we're after (AR1, AR2), which turns detection on."""
        if rclpy.ok():
            self.state_pub.publish(String(data=state))

    def send(self, state, target="", goal_xy=None):
        """Publish a command on mission_state for sm_straight_line.py."""
        msg = MissionState()
        msg.state = state
        msg.current_state = target
        if goal_xy is not None:
            msg.current_goal = PoseStamped()
            msg.current_goal.pose.position.x = float(goal_xy[0])
            msg.current_goal.pose.position.y = float(goal_xy[1])
        if self.pose is not None:
            msg.current_pose = self.pose
        if rclpy.ok():
            self.mission_pub.publish(msg)

    def position(self):
        """(x, y) of the rover from /pose, or None if no pose has arrived."""
        if self.pose is None:
            return None
        return (self.pose.pose.position.x, self.pose.pose.position.y)

    def clear_feedback(self):
        """Throw away old replies so a stale "SLA_DONE" can't end the next drive early."""
        while True:
            try:
                self.feedback.get_nowait()
            except queue.Empty:
                return

    def wait_for_reply(self, replies, timeout_s):
        """Wait for one of `replies` on mission_state. Returns it, or None on timeout,
        abort or shutdown (check self.aborted to tell abort apart)."""
        deadline = None if timeout_s is None else time.time() + timeout_s
        while rclpy.ok() and not self.aborted and (deadline is None or time.time() < deadline):
            try:
                reply = self.feedback.get(timeout=0.1)
            except queue.Empty:
                continue
            if reply in replies:
                return reply
        return None

    def wait(self, done, timeout_s, period_s=0.1):
        """Wait until done() is true. Returns True if it is, False on timeout, abort or
        shutdown (check self.aborted to tell abort apart)."""
        deadline = None if timeout_s is None else time.time() + timeout_s
        while rclpy.ok() and not self.aborted:
            if done():
                return True
            if deadline is not None and time.time() >= deadline:
                return False
            time.sleep(period_s)
        return False


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
# Every state that waits also has an "aborted" outcome: the GUI abort button can end the
# mission from anywhere. They all lead to ABORT.

class Init(smach.State):
    """Initialize Autonomous Navigation: get the target GPS points, convert to x/y.

    Waits for the GUI to publish coordinates on /long_lat_goal_array, converts each target
    into metres from Start (x = east, y = north), the frame the straight-line node drives in,
    and moves gps_to_pose.py's origin to Start so /pose uses that frame too.
      ready:   targets and remaining are set
      aborted: abort button pressed while waiting
    """

    def __init__(self, node, target_names=TARGET_NAMES, set_origin=SET_POSE_ORIGIN_TO_START):
        smach.State.__init__(self, outcomes=["ready", "aborted"],
                             output_keys=["targets", "remaining", "origin_time"])
        self.node = node
        self.target_names = target_names
        self.set_origin = set_origin

    def execute(self, userdata):
        values = self._wait_for_gps_values()
        if values is None:
            return "aborted"
        targets = self._to_xy(values)

        userdata.origin_time = 0.0
        if self.set_origin:
            origin = Pose()
            origin.position.x = float(values[0])     # latitude of Start
            origin.position.y = float(values[1])     # longitude of Start
            self.node.origin_pub.publish(origin)
            userdata.origin_time = time.time()       # poses older than this are in the old frame
            self.node.gui(f"INIT: /pose origin set to Start ({values[0]:.6f}, {values[1]:.6f})")

        for name, (x, y) in targets.items():
            self.node.gui(f"INIT: {name} is {x:.1f} m east, {y:.1f} m north of the start")
        userdata.targets = targets
        userdata.remaining = list(targets.keys())
        return "ready"

    def _wait_for_gps_values(self):
        """Block until the GUI sends the right number of coordinates. None on abort/shutdown."""
        expected = 2 * (1 + len(self.target_names))      # start + each target, lat and lon
        last_warning, last_time = None, 0.0

        while rclpy.ok() and not self.node.aborted:
            values = self.node.gps_values
            if values is not None and len(values) == expected:
                return values
            if values is None:
                warning = (f"INIT: waiting for GPS coordinates on /long_lat_goal_array "
                           f"(Start, {', '.join(self.target_names)})")
            else:
                warning = (f"INIT: got {len(values)} numbers but expected {expected} "
                           f"(Start, {', '.join(self.target_names)}). Check long_lat_goal.csv")
            # Repeat every 5 s, but show a new problem straight away
            if warning != last_warning or time.time() - last_time > 5.0:
                self.node.gui(warning)
                last_warning, last_time = warning, time.time()
            time.sleep(0.1)
        return None

    def _to_xy(self, values):
        """values = [start_lat, start_lon, lat1, lon1, lat2, lon2, ...] in decimal degrees."""
        start_lat, start_lon = values[0], values[1]
        targets = {}
        for i, name in enumerate(self.target_names):
            lat, lon = values[2 + 2 * i], values[3 + 2 * i]
            distance = gps.getDistanceBetweenGPSCoordinates((start_lat, start_lon), (lat, lon))
            heading = gps.getHeadingBetweenGPSCoordinates(start_lat, start_lon, lat, lon)  # 0 = north
            targets[name] = (distance * math.sin(heading), distance * math.cos(heading))
        return targets


class StartGate(smach.State):
    """At Mission Start Location? + Signal Red LED.

    Start is (0, 0) because INIT moved /pose's origin there.
      ok:           within tolerance_m of Start, LED set to red
      not_at_start: too far, or no fresh /pose. LED blue (teleoperate to the start position),
                    waits recheck_s, then back to INIT, which re-reads the coordinates
      aborted:      abort button pressed
    """

    def __init__(self, node, tolerance_m=START_TOLERANCE_M, pose_timeout_s=POSE_TIMEOUT_S,
                 recheck_s=START_RECHECK_S):
        smach.State.__init__(self, outcomes=["ok", "not_at_start", "aborted"],
                             input_keys=["origin_time"])
        self.node = node
        self.tolerance_m = tolerance_m
        self.pose_timeout_s = pose_timeout_s
        self.recheck_s = recheck_s

    def execute(self, userdata):
        # A pose from before INIT moved the origin is in the old frame, so wait for a newer one.
        fresh = self.node.wait(lambda: self.node.pose_time > userdata.origin_time,
                               self.pose_timeout_s)
        if self.node.aborted:
            return "aborted"
        if not fresh:
            return self._not_at_start(f"START_GATE: no /pose within {self.pose_timeout_s:.0f}s. "
                                      f"Is gps_to_pose.py running?")

        x, y = self.node.position()
        distance = math.hypot(x, y)
        if distance > self.tolerance_m:
            return self._not_at_start(f"START_GATE: {distance:.1f} m from Start (limit "
                                      f"{self.tolerance_m:.1f} m). Teleoperate to the start position")

        self.node.set_led("auto")      # red = autonomous (rule 1.e.ii)
        self.node.gui(f"START_GATE: at start ({distance:.1f} m off), LED red")
        return "ok"

    def _not_at_start(self, text):
        self.node.set_led("manual")    # blue = teleoperated
        self.node.gui(text)
        self.node.wait(lambda: False, self.recheck_s)    # pause; returns early on abort
        return "aborted" if self.node.aborted else "not_at_start"


def shortest_route(start, names, targets):
    """Order of `names` with the shortest total straight-line distance from `start`.
    Tries every order."""
    def length(order):
        points = [start] + [targets[name] for name in order]
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return list(min(itertools.permutations(names), key=length))


class LocationSelection(smach.State):
    """Location Selection: pick the next target on the shortest route from the rover.

      target_selected: current_target set, removed from remaining
      all_done:        no targets left
      aborted:         abort button pressed
    """

    def __init__(self, node):
        smach.State.__init__(self,
                outcomes=["target_selected", "all_done", "aborted"],
                input_keys=["targets"],
                io_keys=["remaining"],
                output_keys=["current_target", "approach_attempts"])
        self.node = node

    def execute(self, userdata):
        if self.node.aborted:
            return "aborted"
        if not userdata.remaining:
            self.node.gui("LOCATION_SELECTION: no targets left")
            return "all_done"

        here = self.node.position() or (0.0, 0.0)    # no pose yet: assume we're at Start
        route = shortest_route(here, userdata.remaining, userdata.targets)
        target = route[0]
        userdata.remaining = [name for name in userdata.remaining if name != target]
        userdata.current_target = target
        userdata.approach_attempts = 0
        self.node.gui(f"LOCATION_SELECTION: next target {target} (route: {' -> '.join(route)})")
        return "target_selected"


class Cruise(smach.State):
    """Rover Cruise to Selected Location, then the GNSS[i] / AR Tag[i] branch.

    Turns tag detection on for this target (AR1/AR2 on "state"), asks the obstacle-avoidance
    nodes to drive by publishing START_SL_AVOIDANCE on mission_state, then waits.
    map_path_planner_node.py plans around the GUI's obstacles and publishes GLOBAL_PATH_READY,
    then local_astar.py drives that path.
      arrived: got SLA_AVOIDANCE_DONE
      failed:  got SLA_FAILED / SLA_AVOIDANCE_FAILED, timed out, or ROS shut down
      aborted: abort button pressed (local_astar.py stops itself on the same button)
    """

    def __init__(self, node, timeout_s=CRUISE_TIMEOUT_S, detector_states=TARGET_DETECTOR_STATES):
        smach.State.__init__(self, outcomes=["arrived", "failed", "aborted"],
                             input_keys=["current_target", "targets"])
        self.node = node
        self.timeout_s = timeout_s
        self.detector_states = detector_states

    def execute(self, userdata):
        name = userdata.current_target
        x, y = userdata.targets[name]

        self.node.set_detector_state(self.detector_states.get(name, name))
        self.node.clear_feedback()
        self.node.send("START_SL_AVOIDANCE", name, (x, y))
        self.node.gui(f"CRUISE: driving to {name} at ({x:.1f}, {y:.1f})")

        reply = self.node.wait_for_reply(
            {"SLA_AVOIDANCE_DONE", "SLA_FAILED", "SLA_AVOIDANCE_FAILED"}, self.timeout_s)
        if self.node.aborted:
            self.node.gui(f"CRUISE: aborted on the way to {name}")
            return "aborted"
        if reply == "SLA_AVOIDANCE_DONE":
            self.node.gui(f"CRUISE: arrived at {name}")
            return "arrived"
        if reply in ("SLA_FAILED", "SLA_AVOIDANCE_FAILED"):
            self.node.gui(f"CRUISE: drive to {name} failed")
            return "failed"
        if not rclpy.ok():
            return "failed"    # ROS shut down (Ctrl+C) mid-drive
        self.node.gui(f"CRUISE: no reply for {name} within {self.timeout_s}s")
        return "failed"


class CheckTag(smach.State):
    """Search/Detect Tag, plus Display Tag on GUI.

    Watches the aruco_found topic (from ar_detection_node.py) for up to `window_s` seconds.
      found:     a tag was seen during the window
      not_found: no tag seen in time, or the tag was lost too often while approaching
                 -> back to LOCATION_SELECTION (red line)
      aborted:   abort button pressed
    """

    def __init__(self, node, window_s=CHECK_TAG_WINDOW_S, max_attempts=MAX_APPROACH_ATTEMPTS):
        smach.State.__init__(self, outcomes=["found", "not_found", "aborted"],
                             input_keys=["current_target", "approach_attempts"])
        self.node = node
        self.window_s = window_s
        self.max_attempts = max_attempts

    def execute(self, userdata):
        name = userdata.current_target
        if userdata.approach_attempts >= self.max_attempts:
            self.node.gui(f"CHECK_TAG: lost {name}'s tag {userdata.approach_attempts} times, "
                          f"giving up on it")
            return "not_found"

        start = time.time()
        self.node.gui(f"CHECK_TAG: looking for {name}'s tag")
        # Only sightings since we started looking count, not ones from the drive here.
        seen = self.node.wait(lambda: self.node.last_tag_seen >= start, self.window_s)
        if self.node.aborted:
            return "aborted"
        if seen:
            self.node.gui(f"Tag found: {name}")   # "Display Tag on GUI"
            return "found"
        if rclpy.ok():
            self.node.gui(f"CHECK_TAG: no tag seen at {name} within {self.window_s:.0f}s")
        return "not_found"


def tag_centre_and_area(box):
    """Centre x (pixels) and area (pixels^2) of a bounding box in ar_detection_node.py's corner
    order: top-left, top-right, bottom-left, bottom-right."""
    tl, tr, bl, br = (box[0], box[1]), (box[2], box[3]), (box[4], box[5]), (box[6], box[7])
    centre_x = (tl[0] + tr[0] + bl[0] + br[0]) / 4
    corners = [tl, tr, br, bl]                      # around the edge, for the shoelace formula
    area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(corners, corners[1:] + corners[:1]))) / 2
    return centre_x, area


def homing_command(box, min_area=HOMING_MIN_TAG_AREA, lin_vel=HOMING_LIN_VEL,
                   ang_vel=HOMING_ANG_VEL):
    """What to do about a tag at `box`: (linear, angular) to drive, or None if close enough.
    Same rules as Aimer in aruco_homing_improve.py with sm_grid_search.py's minimum speeds."""
    centre_x, area = tag_centre_and_area(box)
    # Slower as the tag gets bigger (closer): Aimer.scale_velocity
    scale = int(5 * (min_area - area) / min_area) / 5
    offset = centre_x - HOMING_FRAME_WIDTH / 2
    if abs(offset) > HOMING_CENTRE_TOLERANCE_PX:
        turn = max(scale * ang_vel, 0.3)
        return (0.0, turn if offset < 0 else -turn)  # tag on the left: turn left (CCW)
    if area < min_area:
        return (max(scale * lin_vel, 0.2), 0.0)
    return None


class ApproachTag(smach.State):
    """Move Towards the Tag + Stop within 1 m of Target Location (rule 1.e.xvii).

    Homes on the tag's bounding box from ar_detection_node.py (aruco_node/bbox), publishing
    on /drive: turns to face the tag, drives until it looks big enough, then a short final push.
      reached: close enough, stopped
      lost:    no bounding box for lost_after_s, or timed out -> back to CHECK_TAG.
               Counts towards MAX_APPROACH_ATTEMPTS.
      aborted: abort button pressed
    """

    def __init__(self, node, timeout_s=APPROACH_TIMEOUT_S, lost_after_s=HOMING_LOST_AFTER_S,
                 final_push_s=HOMING_FINAL_PUSH_S, min_area=HOMING_MIN_TAG_AREA,
                 lin_vel=HOMING_LIN_VEL, ang_vel=HOMING_ANG_VEL):
        smach.State.__init__(self, outcomes=["reached", "lost", "aborted"],
                             input_keys=["current_target"],
                             io_keys=["approach_attempts"])
        self.node = node
        self.timeout_s = timeout_s
        self.lost_after_s = lost_after_s
        self.final_push_s = final_push_s
        self.min_area = min_area
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

    def execute(self, userdata):
        name = userdata.current_target
        self.node.gui(f"APPROACH_TAG: homing on {name}'s tag")
        result = self._home()
        self.node.drive()                     

        if result == "aborted":
            return "aborted"
        if result == "reached":
            self.node.gui(f"APPROACH_TAG: stopped within 1 m of {name}")
            return "reached"
        userdata.approach_attempts = userdata.approach_attempts + 1
        self.node.gui(f"APPROACH_TAG: {result} at {name} (attempt {userdata.approach_attempts})")
        return "lost"

    def _home(self):
        """Returns "reached", "aborted", or why the tag was lost."""
        start = time.time()
        while rclpy.ok() and not self.node.aborted:
            now = time.time()
            if now - start > self.timeout_s:
                return f"not there within {self.timeout_s:.0f}s"

            # Only boxes from now on count: an older one may be from the drive here.
            box_age = now - self.node.bbox_time
            if self.node.bbox_time < start or box_age > 0.5:
                self.node.drive()               # no current box: hold still
                if now - max(self.node.bbox_time, start) > self.lost_after_s:
                    return "lost sight of the tag"
                time.sleep(0.1)
                continue

            command = homing_command(self.node.bbox, self.min_area, self.lin_vel, self.ang_vel)
            if command is None:              
                end = time.time() + self.final_push_s
                while time.time() < end and rclpy.ok() and not self.node.aborted:
                    self.node.drive(self.lin_vel)
                    time.sleep(0.1)
                return "aborted" if self.node.aborted else "reached"
            self.node.drive(*command)
            time.sleep(0.1)
        return "aborted" if self.node.aborted else "ROS shut down"


class SignalSuccess(smach.State):
    """Flash Green LED, Report Arrival. AWAIT_APPROVAL keeps the LED flashing."""

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["done"], input_keys=["current_target"])
        self.node = node

    def execute(self, userdata):
        self.node.set_led("mission done")
        self.node.gui(f"Goal Point Reached: {userdata.current_target}")
        return "done"


class AwaitApproval(smach.State):
    """Approval + Programming and making adjustments.

    Flashes the LED green until the operator presses Next Task (Bool on /next_state), or
    `timeout_s` passes (out of radio range), then turns it back to red.
      next_objective: targets remain -> back to LOCATION_SELECTION
      finish:         no targets left -> end
      aborted:        abort button pressed
    """

    def __init__(self, node, timeout_s=APPROVAL_TIMEOUT_S, flash_period_s=FLASH_PERIOD_S):
        smach.State.__init__(self, outcomes=["next_objective", "finish", "aborted"],
                             input_keys=["remaining", "current_target"])
        self.node = node
        self.timeout_s = timeout_s
        self.flash_period_s = flash_period_s

    def execute(self, userdata):
        start = time.time()
        self.node.gui(f"AWAIT_APPROVAL: at {userdata.current_target}, waiting for Next Task")

        # Flash: alternate green and off. A press from before we got here doesn't count.
        green = False                   # flipped before the first publish, so green comes first
        approved = False
        while not approved:
            remaining_s = None if self.timeout_s is None else start + self.timeout_s - time.time()
            if remaining_s is not None and remaining_s <= 0:
                break
            green = not green
            self.node.set_led("mission done" if green else "off")
            wait_s = self.flash_period_s if remaining_s is None else min(self.flash_period_s,
                                                                           remaining_s)
            approved = self.node.wait(lambda: self.node.last_next_press >= start, wait_s)
            if self.node.aborted or not rclpy.ok():
                break

        if self.node.aborted:
            return "aborted"
        if approved:
            self.node.gui("AWAIT_APPROVAL: approved")
        elif rclpy.ok():
            self.node.gui(f"AWAIT_APPROVAL: no Next Task within {self.timeout_s:.0f}s "
                          f"(out of radio range?), continuing")
        self.node.set_led("auto")       # back to red: still autonomous
        return "next_objective" if userdata.remaining else "finish"


class Abort(smach.State):
    """Abort: stop and hand control back to the operator.

    sm_straight_line.py stops itself when the abort button is pressed, then refuses to drive
    until restarted, so the operator teleoperates back the way the rover came
    (rule: abort = return the way you came).
    """

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["stopped"])
        self.node = node

    def execute(self, userdata):
        self.node.drive()               # stop, in case we were homing
        self.node.set_led("manual")     # blue = teleoperated
        self.node.gui("ABORT: autonomy stopped. Teleoperate back the way you came")
        return "stopped"


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------
def build_machine(node, start_recheck_s=START_RECHECK_S, cruise_timeout_s=CRUISE_TIMEOUT_S,
                  check_tag_window_s=CHECK_TAG_WINDOW_S, approach_timeout_s=APPROACH_TIMEOUT_S,
                  approval_timeout_s=APPROVAL_TIMEOUT_S):
    """The whole flowchart. The timing arguments default to the settings at the top;
    tests pass shorter ones."""
    sm = smach.StateMachine(outcomes=["tasks_ended", "aborted"])
    sm.userdata.targets = {}
    sm.userdata.remaining = []
    sm.userdata.current_target = None
    sm.userdata.approach_attempts = 0
    sm.userdata.origin_time = 0.0

    with sm:
        smach.StateMachine.add("INIT", Init(node),
                               transitions={"ready": "START_GATE",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("START_GATE", StartGate(node, recheck_s=start_recheck_s),
                               transitions={"ok": "LOCATION_SELECTION",
                                            "not_at_start": "INIT",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("LOCATION_SELECTION", LocationSelection(node),
                               transitions={"target_selected": "CRUISE",
                                            "all_done": "tasks_ended",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("CRUISE", Cruise(node, timeout_s=cruise_timeout_s),
                               transitions={"arrived": "CHECK_TAG",
                                            "failed": "LOCATION_SELECTION",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("CHECK_TAG", CheckTag(node, window_s=check_tag_window_s),
                               transitions={"found": "APPROACH_TAG",
                                            "not_found": "LOCATION_SELECTION",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("APPROACH_TAG", ApproachTag(node, timeout_s=approach_timeout_s),
                               transitions={"reached": "SIGNAL_SUCCESS",
                                            "lost": "CHECK_TAG",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("SIGNAL_SUCCESS", SignalSuccess(node),
                               transitions={"done": "AWAIT_APPROVAL"})
        smach.StateMachine.add("AWAIT_APPROVAL", AwaitApproval(node, timeout_s=approval_timeout_s),
                               transitions={"next_objective": "LOCATION_SELECTION",
                                            "finish": "tasks_ended",
                                            "aborted": "ABORT"})
        smach.StateMachine.add("ABORT", Abort(node),
                               transitions={"stopped": "aborted"})
    return sm


def spin(node):
    """Run ROS callbacks until shutdown. Ctrl+C makes rclpy.spin raise; that's a normal stop."""
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def stop_on_ctrl_c(signum, frame):
    """Ctrl+C: shut ROS down so every waiting loop sees rclpy.ok() == False and the machine
    stops cleanly. Without this, Ctrl+C raises KeyboardInterrupt inside whichever state is
    running and SMACH reports it as a crash. A second Ctrl+C force-quits as usual."""
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.try_shutdown()


def main():
    smach.set_loggers(smach.loginfo, smach.logwarn, lambda msg: None, smach.logerr)
    rclpy.init()
    signal.signal(signal.SIGINT, stop_on_ctrl_c)   # after rclpy.init(), which sets its own handler
    node = RoverNode()
    smach.set_shutdown_check(lambda: not rclpy.ok())

    # ROS callbacks run in this background thread while the machine runs in the main one.
    spin_thread = threading.Thread(target=spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(0.5)

    try:
        result = build_machine(node).execute()
        print("Machine finished with outcome:", result)
    finally:
        # Order matters: stop the spin thread before destroying the node it's using,
        # or the program crashes on exit ("terminate called without an active exception").
        rclpy.try_shutdown()   # Ctrl+C has usually shut ROS down already; shutting down twice raises
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
