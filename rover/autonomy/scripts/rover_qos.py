# rover_qos.py
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# The standard profile for the "state" topic, used in state machine/ar detection/object detection nodes. Adjust as needed, but this is a good starting point for messages that we want to ensure are received by all subscribers, even if they subscribe late.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# You can also define standard profiles for sensors here (NOT USED RIGHT NOW)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)