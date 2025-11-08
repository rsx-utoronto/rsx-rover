from pathlib import Path
from PyQt5.QtWidgets import (
  QWidget, QComboBox, QListWidget, QListWidgetItem, QLabel,
  QLineEdit, QHBoxLayout, QVBoxLayout, QPushButton
)
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from widgets.map_widget import MapOverlay


class EditableComboBox(QComboBox):
  """
  Custom combobox with editable latitude/longitude entry fields.
  Each item contains a label and two text fields for coordinates.
  """

  def __init__(self):
    super().__init__()
    self.setEditable(False)  # Using a QListWidget for custom items
    self.list_widget = QListWidget()
    self.setModel(self.list_widget.model())
    self.setView(self.list_widget)
    self.items_data: list[tuple[QLabel, QLineEdit, QLineEdit]] = []  # Stores references to text edit fields
    self.populate_items()

  def populate_items(self):
    """Populate the combobox with coordinate entry fields."""
    coord_array = ["Start", "GNSS 1", "GNSS 2", "AR 1", "AR 2", "AR 3", "OBJ 1", "OBJ 2"]

    for i in range(len(coord_array)):
      item_widget = QWidget()
      layout = QHBoxLayout()

      label = QLabel(f"Item {coord_array[i]}")  # Static text (not editable)
      text1 = QLineEdit()  # Editable box for latitude
      text2 = QLineEdit()  # Editable box for longitude

      self.items_data.append((label, text1, text2))  # Store references

      layout.addWidget(label)
      layout.addWidget(text1)
      layout.addWidget(text2)
      layout.setContentsMargins(0, 0, 0, 0)
      item_widget.setLayout(layout)

      item = QListWidgetItem(self.list_widget)
      item.setSizeHint(item_widget.sizeHint())
      self.list_widget.addItem(item)
      self.list_widget.setItemWidget(item, item_widget)

  def get_all_data(self):
    """
    Extract all coordinate data from the entry fields.

    Returns:
      list: Flat list of alternating latitude/longitude values
    """
    data = []
    for label, text1, text2 in self.items_data:
      # Add latitude (default to 0 if empty)
      if text1.text() == "":
        data.append(0.0)
      else:
        data.append(float(text1.text()))

      # Add longitude (default to 0 if empty)
      if text2.text() == "":
        data.append(0.0)
      else:
        data.append(float(text2.text()))

    return data


class LngLatEntryBar(QWidget):
  """
  Widget for manual entry of GPS coordinates with map plotting.
  Publishes coordinates to ROS topic when submitted.
  """

  def __init__(self, node: Node, map_overlay: MapOverlay):
    super().__init__()
    self.node = node
    self.longLat_pub = self.node.create_publisher(
      Float32MultiArray, '/long_lat_goal_array', 5
    )
    self.array = Float32MultiArray()
    self.viewer = map_overlay

    # Initialize map layer for points
    self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')

    # ui setup
    layout = QVBoxLayout(self)

    self.combo = EditableComboBox()
    layout.addWidget(self.combo)

    # Add button to collect and publish data
    self.submit_button = QPushButton("Get Data")
    self.submit_button.clicked.connect(self.collect_data) # type: ignore
    self.submit_button.clicked.connect(self.plot_points) # type: ignore
    layout.addWidget(self.submit_button)

    self.setLayout(layout)

  def collect_data(self):
    """Collect coordinate data from UI and publish to ROS topic."""
    data = self.combo.get_all_data()
    # print("Collected Data:")
    self.array.data = data
    self.longLat_pub.publish(self.array)
    # for item in data:
    #   print(item)

  def plot_points(self):
    """Plot the entered coordinates on the map."""
    data = self.combo.get_all_data()
    print("Plotting manually entered points:")

    # Create MapPoint objects from the data (pairs of lat, lng values)
    for i in range(0, len(data), 2):
      if i + 1 < len(data):
        lat = data[i]
        lng = data[i + 1]
        point_name = f"Point {i // 2 + 1}"
        print(f"{point_name}: {lat}, {lng}")
        self.viewer.viewer.goal_points[i // 2].setLatLng([lat, lng])


class LngLatEntryFromFile(QWidget):
  """
  Widget for loading GPS coordinates from a CSV file.
  Reads from 'long_lat_goal.csv' and publishes to ROS topic.
  """

  def __init__(self, node: Node, map_overlay: MapOverlay):
    super().__init__()
    self.node = node
    self.longLat_pub = self.node.create_publisher(Float32MultiArray, '/long_lat_goal_array', 5)
    self.viewer = map_overlay
    self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')
    
    self.array = Float32MultiArray()
    self._ui_setup()
  
  def _ui_setup(self):
    # ui setup
    layout = QVBoxLayout(self)

    self.submit_button = QPushButton("Get Data From File")
    self.submit_button.clicked.connect(self.collect_data) # type: ignore
    self.submit_button.clicked.connect(self.plot_points) # type: ignore
    layout.addWidget(self.submit_button)
    self.setLayout(layout)

  def collect_data(self):
    """Read coordinate data from CSV file and publish to ROS topic."""
    # Read data from the file
    file_path = Path(__file__).parent.parent.parent.parent.resolve() / "long_lat_goal.csv"
    with open(file_path, 'r') as file:
      lines = file.readlines()

    # Process each line and publish
    data = []
    for line in lines:
      t = list(map(float, line.strip().split(',')[1:3]))
      for i in t:
        data.append(i)
    self.array.data = data
    self.longLat_pub.publish(self.array)
    print("Published Data:", data)

  def plot_points(self):
    """Plot the loaded coordinates on the map."""
    data = self.array.data
    print("Plotting points from file data")
    # Create MapPoint objects from the data (pairs of lat, lng values)
    # map_points = []
    for i in range(0, len(data), 2):
      if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
        lat = data[i]
        lng = data[i+1]
        # Create a MapPoint with a radius of 5 and a name based on index
        point_name = f"Point {i//2 + 1}"
        # map_points.append((lat, lng, 5, point_name))
        print(f"{point_name}: {lat}, {lng}")
        self.viewer.viewer.goal_points[i//2].setLatLng([lat, lng])


class LngLatDeliveryEntryFromFile(QWidget):
  """
  Widget for loading delivery GPS coordinates from a CSV file.
  Reads from 'delivery_lat_lon_goal.csv' with named waypoints.
  """

  def __init__(self, map_overlay: MapOverlay):
    super().__init__()
    self.viewer = map_overlay
    self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')
    self._ui_setup()
  
  def _ui_setup(self):
    layout = QVBoxLayout(self)
    self.submit_button = QPushButton("Get Delivery Data From File")
    self.submit_button.clicked.connect(self.collect_data) # type: ignore
    layout.addWidget(self.submit_button)
    self.setLayout(layout)

  def collect_data(self):
    """Read delivery coordinate data from CSV file and plot on map."""
    file_path = Path(__file__).parent.parent.parent.resolve() / "delivery_lat_lon_goal.csv"
    with open(file_path, 'r') as file:
      lines = file.readlines()

    # Process each line and publish
    data = []
    for line in lines:
      t = list(map(float, line.strip().split(',')[1:3]))
      n = line.strip().split(',')[0]
      data.append(n)
      for i in t:
        data.append(i)
    print("Plotting points from file data")
    print(data)

    # Create MapPoint objects from the data (pairs of lat, lng values)
    # map_points = []
    for i in range(0, len(data), 3):
      if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
        lat = data[i+1]
        lng = data[i+2]
        # Create a MapPoint with a radius of 5 and a name based on index
        point_name = data[i]
        # map_points.append((lat, lng, 5, point_name))
        print(f"{point_name}: {lat}, {lng}")
        self.viewer.viewer.add_goal(point_name, lat, lng)
