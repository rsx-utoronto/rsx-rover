#!/usr/bin/env python3
"""
Route-finding state machine

State Machine Diagram: https://lucid.app/lucidspark/de548cc5-ef6a-402f-8165-6ba17ecb4b84/edit?viewport_loc=-1796%2C-218%2C5637%2C3109%2C0_0&invitationId=inv_bac4b471-60e8-4381-884b-c70ed722c39a

Status of each state:
  INIT                [FAKE]         uses the made-up targets in FAKE below
  START_GATE          [PARTLY REAL]  LED is real; "at the start" check is fake
  LOCATION_SELECTION  [PARTLY REAL]  works, but takes targets in list order
  CRUISE              [REAL]         drives via sm_straight_line.py (START_SL -> SLA_DONE/SLA_FAILED)
  CHECK_TAG           [REAL]         watches aruco_found from ar_detection_node.py
  APPROACH_TAG        [PLACEHOLDER]  passes straight through; no homing yet
  SIGNAL_SUCCESS      [REAL]         green LED + GUI message
  AWAIT_APPROVAL      [FAKE]         auto-approves; no Next Task button yet
  Abort               [NOT IMPLEMENTED]

Unresolved questions for TODO:
- Homing: which node should drive up to the tag and stop within 1 m once it’s seen (rule 1.e.xvii)? Right now only the grid search node does homing on command.
- Hill target: should I use straight-line driving, obstacle avoidance (START_SL_AVOIDANCE), or one of the path planners? The rules say not every approach to the hill is drivable.
- Out of radio range: target 2 is behind the hill, so the Next Task button can’t reach the rover there. Should it continue on its own, wait, or drive back into range?
- Abort: the rules allow “stop and return the way you came” for a 20% penalty. Is anyone recording the rover’s path, and what should abort do?
- Target order: shortest path, or does the operator choose?
- Start gate: how do we check the rover is at the start, and how far off counts as not at the start?
"""

import queue
import threading
import time

import rclpy
import smach
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rover.msg import MissionState
from rover_qos import STATE_QOS

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

# Replace with GPS from GUI
FAKE = {
    "at_start": True,                                  # START_GATE position check
    "targets": {"POST1": (10.0, 20.0),                 # x/y in metres from the start
                "POST2": (35.0, -5.0)},
}

CRUISE_TIMEOUT_S = 600    # give up on a drive after 10 minutes
CHECK_TAG_WINDOW_S = 5.0  # after arriving, how long to look for the tag before giving up


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

        self.create_subscription(MissionState, "mission_state", self._feedback_callback, 10)
        self.create_subscription(PoseStamped, "/pose", self._pose_callback, 10)
        # ar_detection_node.py publishes True on aruco_found whenever it sees a tag
        self.create_subscription(Bool, "aruco_found", self._aruco_callback, 10)

        self.feedback = queue.Queue()
        self.pose = None          # latest PoseStamped from /pose, None until one arrives
        self.last_tag_seen = 0.0  # time.time() when a tag was last seen, 0 = never

    def _feedback_callback(self, msg):
        self.feedback.put(msg.state)

    def _pose_callback(self, msg):
        self.pose = msg

    def _aruco_callback(self, msg):
        if msg.data:
            self.last_tag_seen = time.time()

    # helpers
    def set_led(self, mode):
        """mode: "auto" (red), "manual" (blue), "mission done" (green). See led_light.py."""
        self.led_pub.publish(String(data=mode))

    def gui(self, text):
        """Status line for the GUI, and printed locally."""
        print(text)
        self.gui_pub.publish(String(data=text))

    def clear_feedback(self):
        """Throw away old replies so a stale "SLA_DONE" can't end the next drive early."""
        while True:
            try:
                self.feedback.get_nowait()
            except queue.Empty:
                return


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class Init(smach.State):
    """Initialize Autonomous Navigation: get the target GPS points, convert to x/y.  [FAKE]"""

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["ready"],
                             output_keys=["targets", "remaining"])
        self.node = node

    def execute(self, userdata):
        self.node.gui("INIT: getting target coordinates (fake)")
        # TODO: wait for GPS points on /long_lat_goal_array and convert each to x/y.
        userdata.targets = dict(FAKE["targets"])
        userdata.remaining = list(FAKE["targets"].keys())
        return "ready"


class StartGate(smach.State):
    """Start Gate Verification + Signal Red LED.  [PARTLY REAL: LED real, position check fake]"""

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["ok", "not_at_start"])
        self.node = node

    def execute(self, userdata):
        # TODO (real): check the rover is actually at the start location.
        if not FAKE["at_start"]:
            self.node.gui("START_GATE: not at start, re-adjust")
            return "not_at_start"
        self.node.set_led("auto")      # red = autonomous (rule 1.e.ii)
        self.node.gui("START_GATE: at start, LED red")
        return "ok"


class LocationSelection(smach.State):
    """Pick the next target to drive to.  [PARTLY REAL: takes targets in list order]"""

    def __init__(self, node):
        smach.State.__init__(self,
                outcomes=["target_selected", "all_done"],
                io_keys=["remaining"],
                output_keys=["current_target"])
        self.node = node

    def execute(self, userdata):
        if not userdata.remaining:
            self.node.gui("LOCATION_SELECTION: no targets left")
            return "all_done"
        # TODO (real): pick by shortest path instead of list order
        target = userdata.remaining.pop(0)
        userdata.current_target = target   # output key: write only, so use the local copy below
        self.node.gui(f"LOCATION_SELECTION: next target {target}")
        return "target_selected"


class Cruise(smach.State):
    """Rover Cruise to Selected Location.  [REAL]

    Asks the straight-line node (sm_straight_line.py) to drive by publishing
    START_SL on mission_state, then waits for its reply.
      arrived: got SLA_DONE
      failed:  got SLA_FAILED, timed out, or ROS shut down
    """

    def __init__(self, node, timeout_s=CRUISE_TIMEOUT_S):
        smach.State.__init__(self, outcomes=["arrived", "failed"],
                             input_keys=["current_target", "targets"])
        self.node = node
        self.timeout_s = timeout_s

    def execute(self, userdata):
        name = userdata.current_target
        x, y = userdata.targets[name]

        msg = MissionState()
        msg.state = "START_SL"
        msg.current_state = name
        msg.current_goal = PoseStamped()
        msg.current_goal.pose.position.x = float(x)
        msg.current_goal.pose.position.y = float(y)
        if self.node.pose is not None:
            msg.current_pose = self.node.pose

        self.node.clear_feedback()
        self.node.mission_pub.publish(msg)
        self.node.gui(f"CRUISE: driving to {name} at ({x:.1f}, {y:.1f})")

        deadline = time.time() + self.timeout_s
        while time.time() < deadline and rclpy.ok():
            # TODO: check self.preempt_requested() here so an operator abort can stop the drive.
            try:
                reply = self.node.feedback.get(timeout=0.1)
            except queue.Empty:
                continue
            if reply == "SLA_DONE":
                self.node.gui(f"CRUISE: arrived at {name}")
                return "arrived"
            if reply == "SLA_FAILED":
                self.node.gui(f"CRUISE: drive to {name} failed")
                return "failed"

        self.node.gui(f"CRUISE: no reply for {name} within {self.timeout_s}s")
        return "failed"


class CheckTag(smach.State):
    """The "Tag found?" diamond, plus Display Tag on GUI.  [REAL]

    After CRUISE arrives at the target's coordinates, watch the aruco_found topic
    (from ar_detection_node.py) for up to `window_s` seconds.
      found:     a tag was seen during the window
      not_found: no tag seen in time -> abort back to LOCATION_SELECTION (red line)
    """

    def __init__(self, node, window_s=CHECK_TAG_WINDOW_S):
        smach.State.__init__(self, outcomes=["found", "not_found"],
                             input_keys=["current_target"])
        self.node = node
        self.window_s = window_s

    def execute(self, userdata):
        name = userdata.current_target
        start = time.time()
        self.node.gui(f"CHECK_TAG: looking for {name}'s tag")

        while time.time() - start < self.window_s and rclpy.ok():
            if self.node.last_tag_seen >= start:      # seen since we started looking
                self.node.gui(f"Tag found: {name}")   # "Display Tag on GUI"
                return "found"
            time.sleep(0.1)

        self.node.gui(f"CHECK_TAG: no tag seen at {name} within {self.window_s:.0f}s")
        return "not_found"


class ApproachTag(smach.State):
    """Move Towards the Tag + Stop within acceptable range.  [PLACEHOLDER]

      reached: within 1 m of the post (rule 1.e.xvii)
      lost:    lost sight of the tag -> back to CHECK_TAG
    """

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["reached", "lost"],
                             input_keys=["current_target"])
        self.node = node

    def execute(self, userdata):
        self.node.gui(f"APPROACH_TAG: homing on {userdata.current_target} (placeholder)")
        # TODO: no node currently homes on a tag when told to over
        #   mission_state.
        return "reached"


class SignalSuccess(smach.State):
    """Signal Success (LED, message).  [REAL]"""

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["done"], input_keys=["current_target"])
        self.node = node

    def execute(self, userdata):
        self.node.set_led("mission done")
        self.node.gui(f"Goal Point Reached: {userdata.current_target}")
        # NOTE: rule 1.e.ii wants FLASHING green; led_light.py currently sets solid green.
        return "done"


class AwaitApproval(smach.State):
    """Approval / Programming and making adjustments.  [FAKE: auto-approves]

      next_objective: targets remain -> back to LOCATION_SELECTION
      finish:         no targets left -> end
    """

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["next_objective", "finish"],
                             input_keys=["remaining"])
        self.node = node

    def execute(self, userdata):
        self.node.gui("AWAIT_APPROVAL: auto-approving (fake)")
        # TODO (real): wait for the Next Task button (Bool on /next_state from the GUI).
        #   Target 2 is behind the hill, out of radio range (rule 1.e.xvi), so the
        #   button may never arrive there.
        time.sleep(1.0)                 # leave the green LED on for a moment
        self.node.set_led("auto")       # back to red: still autonomous
        return "next_objective" if userdata.remaining else "finish"


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------
def build_machine(node):
    sm = smach.StateMachine(outcomes=["tasks_ended"])
    sm.userdata.targets = {}
    sm.userdata.remaining = []
    sm.userdata.current_target = None

    with sm:
        smach.StateMachine.add("INIT", Init(node),
                               transitions={"ready": "START_GATE"})
        smach.StateMachine.add("START_GATE", StartGate(node),
                               transitions={"ok": "LOCATION_SELECTION",
                                            "not_at_start": "INIT"})
        smach.StateMachine.add("LOCATION_SELECTION", LocationSelection(node),
                               transitions={"target_selected": "CRUISE",
                                            "all_done": "tasks_ended"})
        smach.StateMachine.add("CRUISE", Cruise(node),
                               transitions={"arrived": "CHECK_TAG",
                                            "failed": "LOCATION_SELECTION"})
        smach.StateMachine.add("CHECK_TAG", CheckTag(node),
                               transitions={"found": "APPROACH_TAG",
                                            "not_found": "LOCATION_SELECTION"})
        smach.StateMachine.add("APPROACH_TAG", ApproachTag(node),
                               transitions={"reached": "SIGNAL_SUCCESS",
                                            "lost": "CHECK_TAG"})
        smach.StateMachine.add("SIGNAL_SUCCESS", SignalSuccess(node),
                               transitions={"done": "AWAIT_APPROVAL"})
        smach.StateMachine.add("AWAIT_APPROVAL", AwaitApproval(node),
                               transitions={"next_objective": "LOCATION_SELECTION",
                                            "finish": "tasks_ended"})
    return sm


def main():
    smach.set_loggers(smach.loginfo, smach.logwarn, lambda msg: None, smach.logerr)
    rclpy.init()
    node = RoverNode()
    smach.set_shutdown_check(lambda: not rclpy.ok())

    # ROS callbacks run in this background thread while the machine runs in the main one.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(0.5)

    try:
        result = build_machine(node).execute()
        print("Machine finished with outcome:", result)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()