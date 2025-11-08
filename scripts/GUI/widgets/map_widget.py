from pathlib import Path
from PyQt5.QtWidgets import QWidget, QGridLayout
from sensor_msgs.msg import NavSatFix
from calian_gnss_ros2_msg.msg import GnssSignalStatus # type: ignore
from rclpy.node import Node

from map_viewer import map_viewer

CACHE_DIR = Path(__file__).parent.parent.resolve() / "tile_cache"

class MapOverlay(QWidget):
  """
  Displays a map and shows the rover's current
  position and heading based on GPS data.
  """

  def __init__(self, node: Node):
    super().__init__()
    self.node = node
    self.centre_on_rover = False
    
    self.viewer = map_viewer.MapViewer()
    self.viewer.set_map_server(
        str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
    )
    
    self.setLayout(self._init_layout())
    
    self.node.create_subscription(NavSatFix, '/calian_gnss/gps', self.update_gps_coordinates, 10)
    self.node.create_subscription(GnssSignalStatus, '/calian_gnss/gps_extended', self.update_gps_heading, 10)

  def _init_layout(self) -> QGridLayout:
    """Initialize the layout with map viewer."""
    layout = QGridLayout()
    layout.addWidget(self.viewer)
    return layout

  def update_gps_coordinates(self, msg: NavSatFix):
    """Update rover position from GPS coordinates.
    
    Args:
        msg: NavSatFix message containing latitude and longitude
    """
    gps_point = (msg.latitude, msg.longitude)
    self.viewer.set_robot_position(msg.latitude, msg.longitude)
    
    if self.centre_on_rover:
        self.viewer.center_on_gps(gps_point)

  def update_gps_heading(self, msg: GnssSignalStatus):
    """Update rover heading from GPS data.
    
    Args:
        msg: GnssSignalStatus message containing heading information
    """
    self.viewer.headingSignal.emit(msg.heading)

  def clear_map(self):
    """Clear all lines and paths from the map."""
    self.viewer.clear_lines()

