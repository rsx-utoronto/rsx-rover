import math
import time
from collections import deque

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ---- Settings you can tune ----
WINDOW_SECONDS = 1.5   # how much history to look at
MIN_AMPLITUDE = 0.5    # how big the finger swing must be
MIN_FLIPS = 7          # number of open<->closed switches needed
MAX_PALM_MOVE = 0.8    # palm may drift this many hand-sizes
MAX_THUMB_MOVE = 0.5   # thumb may move this many hand-sizes (relative to palm)
HOLD_SECONDS = 1.0     # keep reporting True briefly after detection

FINGERTIPS = [8, 12, 16, 20]
PALM_IDS = [0, 5, 9, 13, 17]


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def spread(values):
    return max(values) - min(values)


def measure(hand, w, h):
    """Turn one frame's landmarks into the numbers we care about."""
    lm = hand.landmark
    pt = lambda i: (lm[i].x * w, lm[i].y * h)  # convert to pixels

    wrist = pt(0)
    hand_size = dist(wrist, pt(9))

    # 1) Curl: average fingertip-to-wrist distance, divided by hand size
    curl = sum(dist(wrist, pt(t)) for t in FINGERTIPS) / 4 / hand_size

    # 2) Palm center = average of wrist + the four finger base knuckles
    palm_x = sum(pt(i)[0] for i in PALM_IDS) / len(PALM_IDS)
    palm_y = sum(pt(i)[1] for i in PALM_IDS) / len(PALM_IDS)

    # 3) Thumb tip position relative to the palm, in hand-sizes
    thumb = pt(4)
    thumb_dx = (thumb[0] - palm_x) / hand_size
    thumb_dy = (thumb[1] - palm_y) / hand_size

    return curl, palm_x, palm_y, thumb_dx, thumb_dy, hand_size


def analyze(history):
    """Look at the recent history and decide if it's a 'come here'."""
    if len(history) < 10:
        return False, 0.0, 0

    curls = [s["curl"] for s in history]
    amplitude = spread(curls)

    # Count swings between "high" and "low" with a dead zone in the middle
    mid = (max(curls) + min(curls)) / 2
    band = amplitude * 0.2
    state, flips = None, 0
    for c in curls:
        if c > mid + band:
            new = "high"
        elif c < mid - band:
            new = "low"
        else:
            continue
        if state is not None and new != state:
            flips += 1
        state = new

    # Stillness checks
    avg_size = sum(s["size"] for s in history) / len(history)
    palm_move = max(spread([s["px"] for s in history]),
                    spread([s["py"] for s in history])) / avg_size
    thumb_move = max(spread([s["tdx"] for s in history]),
                     spread([s["tdy"] for s in history]))

    detected = (
        amplitude >= MIN_AMPLITUDE
        and flips >= MIN_FLIPS
        and palm_move <= MAX_PALM_MOVE
        and thumb_move <= MAX_THUMB_MOVE
    )
    return detected, amplitude, flips


class GestureNode(Node):
    def __init__(self):
        super().__init__("come_here_gesture_node")

        # Parameters: settings you can change when launching the node
        self.declare_parameter("show_window", True)
        self.declare_parameter("camera_index", 0)
        self.show_window = self.get_parameter("show_window").value

        # Publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(Bool, "come_here", 10)

        # Mediapipe and state, created once
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.history = deque()
        self.last_detected = 0.0

        self.cap = cv2.VideoCapture(self.get_parameter("camera_index").value)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")

        # Call process_frame about 30 times per second
        self.timer = self.create_timer(1 / 30, self.process_frame)
        self.get_logger().info("Come-here gesture node started")

    def process_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Could not read from camera")
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        now = time.time()

        if result.multi_hand_landmarks:
            hand = result.multi_hand_landmarks[0]
            self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)

            curl, px, py, tdx, tdy, size = measure(hand, w, h)
            self.history.append({"t": now, "curl": curl, "px": px, "py": py,
                                 "tdx": tdx, "tdy": tdy, "size": size})

            # Throw away frames older than the window
            while self.history and now - self.history[0]["t"] > WINDOW_SECONDS:
                self.history.popleft()

            detected, amplitude, flips = analyze(self.history)
            if detected:
                self.last_detected = now

            cv2.putText(frame, f"curl {curl:.2f}  swing {amplitude:.2f}  flips {flips}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        else:
            self.history.clear()  # no hand = forget the past

        come_here = (now - self.last_detected) < HOLD_SECONDS

        # Publish the result for other nodes
        msg = Bool()
        msg.data = come_here
        self.publisher.publish(msg)

        if self.show_window:
            if come_here:
                cv2.putText(frame, "COME HERE", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.imshow("Come Here Gesture", frame)
            cv2.waitKey(1)  # required for the window to refresh

    def destroy_node(self):
        self.cap.release()
        self.hands.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = GestureNode()
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