from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QTextEdit
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont, QTextCursor
from std_msgs.msg import Bool, String
from rclpy.node import Node


class ObjectBar(QWidget):
  """Widget for displaying mallet and water bottle detection status."""

  update_mallet_signal = pyqtSignal(bool)
  update_bottle_signal = pyqtSignal(bool)

  def __init__(self, node: Node):
    super().__init__()
    self.init_ui()
    self.node = node
    # Connect signals to the corresponding update methods
    self.update_mallet_signal.connect(self.update_mallet)
    self.update_bottle_signal.connect(self.update_bottle)

    # Initialize ROS subscribers
    # rospy.Subscriber('mallet_detected', Bool, self.mallet_callback)
    # rospy.Subscriber('waterbottle_detected', Bool, self.bottle_callback)
    node.create_subscription(Bool, 'mallet_detected', self.mallet_callback, 10)
    node.create_subscription(Bool, 'waterbottle_detected', self.bottle_callback, 10)

    self.received_strings = []

  def _init_ui(self):
    # Create a label
    self.label_mallet = QLabel("Mallet not found", self)
    self.label_mallet.setAlignment(Qt.AlignCenter) # type: ignore
    self.label_mallet.setStyleSheet("""
      background-color: #808080; 
      color: white;  
      border: 2px solid black;  
      border-radius: 10px; 
      padding: 10px; 
    """)
    self.label_mallet.setFont(QFont("Arial", 72, QFont.Bold))

    # Create a scrollable box for received strings
    self.label_bottle = QLabel("Waterbottle not found", self)
    self.label_bottle.setAlignment(Qt.AlignCenter) # type: ignore
    self.label_bottle.setStyleSheet("""
      background-color: #808080; 
      color: white;
      border: 2px solid black;  
      border-radius: 10px; 
      padding: 10px; 
    """)
    self.label_bottle.setFont(QFont("Arial", 72, QFont.Bold))

    # Layout
    layout = QHBoxLayout()
    layout.addWidget(self.label_mallet)
    layout.addWidget(self.label_bottle)
    self.setLayout(layout)

  def mallet_callback(self, msg: Bool):
    """ROS callback for mallet detection.
    
    Args:
        msg: ROS Bool message
    """
    self.update_mallet_signal.emit(msg.data)

  def bottle_callback(self, msg: Bool):
    """ROS callback for water bottle detection.
    
    Args:
        msg: ROS Bool message
    """
    self.update_bottle_signal.emit(msg.data)

  def update_mallet(self, found: bool):
    """Update mallet detection status.
    
    Args:
        found: Whether mallet was detected
    """
    text = "Mallet Found" if found else "Mallet not found"
    self.label_mallet.setText(text)
    bg_color = "#4CAF50" if found else "#FF5252"
    self.label_mallet.setStyleSheet(f"""
      background-color: {bg_color}; 
      color: white;  
      border: 2px solid black;  
      border-radius: 10px;  
      padding: 10px;  
    """)

  def update_bottle(self, found: bool):
    """Update water bottle detection status.
    
    Args:
        found: Whether water bottle was detected
    """
    text = "Waterbottle Found" if found else "Waterbottle not found"
    self.label_bottle.setText(text)
    bg_color = "#4CAF50" if found else "#FF5252"
    self.label_bottle.setStyleSheet(f"""
      background-color: {bg_color}; 
      color: white;  
      border: 2px solid black;  
      border-radius: 10px;  
      padding: 10px;  
    """)
