from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QTextEdit, QPushButton
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont, QTextCursor
from std_msgs.msg import String, Bool
from rclpy.node import Node


class StatusTerminal(QWidget):
  """Terminal widget for displaying status messages."""

  update_status_signal = pyqtSignal(str)

  def __init__(self):
    super().__init__()
    self.received_strings = []
    self.strlength = -1
    
    self._init_ui()
    # Connect signals to the corresponding update methods
    self.update_status_signal.connect(self.update_string_list)

  def _init_ui(self):
    # Create a scrollable box for received strings
    self.string_list = QTextEdit(self)
    self.string_list.setReadOnly(True)
    self.string_list.setStyleSheet("""
        background-color: #FFFFFF; 
        color: black; 
        border: 2px solid black;  
        padding: 5px; 
    """)

    self.clear_button = QPushButton("Clear")
    self.clear_button.clicked.connect(self.clear_text) # type: ignore
    self.clear_button.setStyleSheet("""
        background-color: #FF5252;
        color: white;
        border: 2px solid black;
        border-radius: 10px;
        padding: 10px;
    """)

    # Layout
    layout = QVBoxLayout()
    layout.addWidget(self.clear_button)
    layout.addWidget(self.string_list)
    self.setLayout(layout)

  def clear_text(self):
    """Clear all text from the terminal."""
    self.string_list.clear()
    self.received_strings = []
    self.strlength = -1
    self.string_list.setPlainText("")
    self.string_list.moveCursor(QTextCursor.Start)

  def string_callback(self, msg: String):
    """ROS callback for status messages.
    
    Args:
        msg: ROS String message with status text
    """
    self.update_status_signal.emit(msg.data.strip())

  def update_string_list(self, new_string: str):
    """Update the terminal with a new status message.
    
    Args:
        new_string: New status message to append
    """
    self.received_strings.append(new_string)
    cursor_pos = self.string_list.textCursor().position()
    
    self.string_list.append(new_string)
    
    # Auto-scroll to end if cursor was already at the end
    if cursor_pos < self.strlength - len(self.received_strings[-1]):
        self.string_list.moveCursor(QTextCursor.End)
    
    self.strlength += len(new_string) + 1


class StateMachineStatus(QWidget):
  """
  Shows the current state machine status using colored labels
  """

  update_label_signal = pyqtSignal(str)

  def __init__(self, node: Node):
    """Initialize the state machine status widget.
    
    Args:
        node: ROS2 node for creating subscriptions
    """
    super().__init__()
    self.node = node
    self._init_ui()
    
    # Connect signal
    self.update_label_signal.connect(self.update_label)
    
    # Subscribe to ROS topic
    node.create_subscription(String, '/led_colour', self.callback, 10)

  def _init_ui(self):
    # Create a label
    self.label = QLabel("Uninitialized LED", self)
    self.label.setAlignment(Qt.AlignCenter) # type: ignore
    self.label.setStyleSheet("""
      background-color: #808080; 
      color: white;  
      border: 2px solid black; 
      border-radius: 10px; 
      padding: 10px; 
    """)
    self.label.setFont(QFont("Arial", 16, QFont.Bold))

    # Layout
    layout = QVBoxLayout()
    layout.addWidget(self.label)
    self.setLayout(layout)

  def callback(self, msg: String):
    """ROS callback for color changes.
    
    Args:
        msg: ROS String message with color
    """
    self.update_label_signal.emit(msg.data)

  def update_label(self, color: str):
    """Update the label based on color.
    
    Args:
        color: one of ('red', 'green', 'yellow')
    """
    color_messages = {
      'red': 'Red status message',
      'green': 'Green status message',
      'yellow': 'Yellow status message',
    }
    if color not in color_messages:
      color = "white"
    message = color_messages.get(color, 'Unknown status')
    self.label.setText(message)
    self.label.setStyleSheet(f"""
      background-color: {color};  
      color: black;           
      border: 2px solid black;
      border-radius: 10px;
      padding: 10px;
    """)



class ArucoWidget(QWidget):
  """Widget for displaying ArUco marker detection with name list."""

  # Define signals to communicate with the main thread
  update_label_signal = pyqtSignal(bool)
  update_list_signal = pyqtSignal(str)

  def __init__(self, node: Node, enable_status_box=False):
    super().__init__()
    self.enable_status_box = enable_status_box
    
    self.init_ui()

    # Connect signals to the corresponding update methods
    self.update_label_signal.connect(self.update_label)
    self.update_list_signal.connect(self.update_string_list)

    # Initialize ROS subscribers
    node.create_subscription(Bool, 'aruco_found', self.bool_callback, 10)
    node.create_subscription(String, 'aruco_name', self.string_callback, 10)

    self.received_strings = []

  def init_ui(self):
    # Create a label
    self.label = QLabel("Aruco not found", self)
    self.label.setAlignment(Qt.AlignCenter) # type: ignore
    self.label.setStyleSheet("""
        background-color: #808080; 
        color: white;  
        border: 2px solid black;  
        border-radius: 10px; 
        padding: 10px; 
    """)
    self.label.setFont(QFont("Arial", 72, QFont.Bold))

    # Create a scrollable box for received strings
    self.string_list = QTextEdit(self)
    self.string_list.setReadOnly(True)
    self.string_list.setStyleSheet("""
        background-color: #FFFFFF; 
        color: black; 
        border: 2px solid black;  
        padding: 5px; 
    """)

    # Layout
    layout = QVBoxLayout()
    layout.addWidget(self.label)
    if self.enable_status_box:
      layout.addWidget(self.string_list)
    self.setLayout(layout)

  def bool_callback(self, msg):
    # Emit signal to update the label in the main thread
    self.update_label_signal.emit(msg.data)

  def string_callback(self, msg):
    # Emit signal to update the list in the main thread
    self.update_list_signal.emit(msg.data.strip())

  def update_label(self, found):
    # Update the label in the main thread
    if found:
      self.label.setText("Aruco Found")
      bg_color = "#4CAF50" 
    else:
      self.label.setText("Aruco not found")
      bg_color = "#FF5252" 
    self.label.setStyleSheet(f"""
      background-color: {bg_color}; 
      color: white;  
      border: 2px solid black;  
      border-radius: 10px;  
      padding: 10px;  
    """)

  def update_string_list(self, new_string):
    # Append the string to the list in the main thread
    self.received_strings.append(new_string)
    self.string_list.setPlainText("\n".join(self.received_strings))
    self.string_list.moveCursor(QTextCursor.End)
