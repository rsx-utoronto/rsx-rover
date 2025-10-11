"""
MapViewer - contains methods to initialize and perform actions 
on a pyqtlet map 

The marker widget shows a collection of Marker objects onscreen 
relative to the robot's position.
"""

import math

import os
from dataclasses import dataclass
from pathlib import Path

import map_js_snnipets

os.environ['QT_API'] = 'pyqt5'
from pyqtlet2 import L, MapWidget

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication

CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"

"""
HELPERS
"""
def dist(latlong1, latlong2):
	if not latlong1 or not latlong2:
		return math.inf
	return math.sqrt((latlong1[0] - latlong2[0])**2 + (latlong1[1] - latlong2[1])**2)

def resolve_path(short_path):
	"""append the parent folder's abs path to the short path"""
	return str(Path(__file__).parent.resolve() / short_path)


@dataclass
class MapPoint:
	latitude: float
	longitude: float
	radius: float
	name: str


class Locations():
	"""constant locations, lat long  pairs"""
	engineering = [(38.380691+38.439607)/2,(-110.829048+-110.751286)/2]
	mdrs = [38.406458, -110.791903]


class MapViewer(QWidget):
	# Icon paths
	ROVER_ICON = resolve_path("icons/rover.png")
	GOAL_NAME_PATHS = {
    "Start": 	resolve_path("icons/map_icon_start.png"),
    "GNSS 1":	resolve_path("icons/map_icon_GNSS.png"),
    "GNSS 2":	resolve_path("icons/map_icon_GNSS.png"),
    "AR 1":  	resolve_path("icons/map_icon_AR.png"),
    "AR 2":  	resolve_path("icons/map_icon_AR.png"),
    "AR 3":		resolve_path("icons/map_icon_AR.png"),
    "OBJ 1":	resolve_path("icons/map_icon_OBJ.png"),
    "OBJ 2":	resolve_path("icons/map_icon_OBJ.png"),
	}
	
	# Thresholds
	ROBOT_REDRAW_THRESHOLD = 0.000001
	LINE_DRAW_THRESHOLD = 0.00001
	
	# Default settings
	DEFAULT_ZOOM = 18

	headingSignal = pyqtSignal(float)

	def __init__(self, *args, **kwargs) -> None:
		super().__init__(*args, **kwargs)

		self._init_map_widget()
		self._init_map_config()
		self._init_markers(Locations.engineering)

		# signal to be used by other components
		self.headingSignal.connect(self.set_robot_rotation)
		
		# 'initialize' tile layer
		self.tile_layer = None

		# initialize dictionaries for storing info
		self.map_points: dict[str, list[MapPoint]] = {}
		self.points_line: dict[str, L.polyline] = {}
		self.points_layer: dict[str, L.layerGroup] = {}
		self.selected_marker: dict[str, L.circle] = {}
		self.show_selected_marker: dict[str, bool] = {}
		self.circle_color: dict[str, str] = {}

	"""
	INIT METHODS
	"""
	def _init_map_widget(self) -> None:
		self.mapWidget = MapWidget()
		layout = QVBoxLayout()
		layout.setContentsMargins(5, 0, 0, 0)
		layout.addWidget(self.mapWidget)
		self.setLayout(layout)
	
	def _init_map_config(self) -> None:
		# create pyqtlet2 map
		self.map = L.map(
			self.mapWidget,
			{
				"doubleClickZoom": False,
				"maxBountsViscosity": 1.0,
			}
		)

		# initialize view and cursor
		self.map.setView(Locations.mdrs, self.DEFAULT_ZOOM)
		map_js_snnipets.init_map_cursor(self.map)

		# add scale to the map
		map_js_snnipets.init_map_scale(self.map)
	
	def _init_markers(self, startup_location: list[float]) -> None:
		
		# --- robot point 
		robot_startup_location = startup_location

		# Get absolute path to the rover icon
		rover_icon_path = self.ROVER_ICON

		# Create the marker first
		self.robot = L.marker(robot_startup_location, {
			'rotationAngle': 0,
			'rotationOrigin': '13px 13px'
		})

		self.robot.bindPopup("Robot")
		self.robot.addTo(self.map)
		map_js_snnipets.init_map_set_robot_icon(self.map, rover_icon_path, self.robot.jsName)
		
		# --- goal points
		self.goal_points: list[L.marker] = []

		# initally there are 8 goals
		for k, v in self.GOAL_NAME_PATHS.items():
			goal_marker = L.marker(robot_startup_location, {
				'rotationAngle': 0,
				'rotationOrigin': '10px 12px',
				'title': k
				# 'icon': icon
			})
			self.goal_points.append(goal_marker)
			goal_marker.addTo(self.map)
			map_js_snnipets.map_marker_set_icon(self.map, v, goal_marker.jsName)
		
		# initialize robot line
		self.last_moved_robot_position = None
		self.last_drawn_robot_position = robot_startup_location
		self.robot_line = L.polyline([], {'color': 'red'})
		self.robot_line.addTo(self.map)
	
	"""
	MAP SETTINGS
	"""
	def set_map_server(self, tile_url: str, layer_count: int) -> None:
		"""Set the url of the tile images' source """
		# remove current tile layer
		if self.tile_layer:
			self.map.removeLayer(self.tile_layer)

		max_layer_depth = layer_count - 1

		# add new tile layer
		self.tile_layer = L.tileLayer(
			tile_url,
			{
				"gridColor": "#FFFFFF",
				"maxNativeZoom": max_layer_depth,
				"maxZoom": max_layer_depth + 5,
			},
		)
		self.tile_layer.addTo(self.map)
	
	"""
	GOAL AND MARKER
	"""
	def add_goal(self, name: str, lat: float, long: float) -> None:
		"""Add a goal to the map."""
		if name == "Start":
			self.goal_points[0].setLatLng([lat, long])
		else:
			path = resolve_path("icons/map_icon_base.png")
			self.goal_points.append(L.marker([lat, long], {
				'rotationAngle': 0,
				'rotationOrigin': '10px 12px',
				'title': name
			}))
			self.goal_points[-1].addTo(self.map)
			map_js_snnipets.map_marker_set_icon(self.map, path, self.goal_points[-1].jsName)
			self.goal_points[-1].bindPopup(name)
	
	def add_popup_marker(self, location: list[float], popup_message: str) -> None:
		popup_marker = L.marker(location)
		popup_marker.bindPopup(popup_message)
		popup_marker.addTo(self.map)

	"""
	POINT LAYER
	"""
	def add_point_layer(self, layer_name: str, circle_color: str, line_color: str, selected_color: str) -> None:
		self.points_line[layer_name] = L.polyline([], {'color': line_color})
		self.points_layer[layer_name] = L.layerGroup()
		self.selected_marker[layer_name] = L.circle([0, 0], 2, {'color': selected_color, 'weight': 8})
		self.show_selected_marker[layer_name] = True
		self.circle_color[layer_name] = circle_color

		self.points_line[layer_name].addTo(self.map)
		self.points_layer[layer_name].addTo(self.map)

	def draw_line_through_points(self, layer: str, points: list[MapPoint]) -> None:
		markers_line = self.points_line[layer]
		latlngs = []
		if self.last_drawn_robot_position is not None:
			latlngs.append(self.last_drawn_robot_position)
		latlngs.extend([[p.latitude, p.longitude] for p in points])
		# used runjavascript with index 1???
		map_js_snnipets.map_marker_set_lat_lng_points(self.map, latlngs, markers_line.jsName)

	def set_points(self, layer: str, points: list[MapPoint]) -> None:
		self.map_points[layer] = points

		# draw the new circles
		self.points_layer[layer].clearLayers()
		for point in points:
			c = L.circle([point.latitude, point.longitude], point.radius, {'weight': 4})
			if point.name:
				map_js_snnipets.map_layer_bind_tooltip(c, point.name, {'permanent': True, 'direction': 'down'})
			self.points_layer[layer].addLayer(c)

		self.draw_line_through_points(layer, points)

	def set_highlighted_point(self, layer: str, index: int) -> None:
		lat = self.map_points[layer][index].latitude
		long = self.map_points[layer][index].longitude
		radius = self.map_points[layer][index].radius

		map_js_snnipets.map_marker_set_lat_lng_point(self.map, [lat, long], self.selected_marker[layer].jsName)
		map_js_snnipets.map_marker_set_radius(self.map, radius + 2, self.selected_marker[layer].jsName)
		if not self.show_selected_marker[layer]:
			self.selected_marker[layer].addTo(self.map)
		self.show_selected_marker[layer] = True
	
	def unhighlight_point(self, layer: str) -> None:
		if self.show_selected_marker[layer]:
			self.selected_marker[layer].removeFrom(self.map)
			self.show_selected_marker[layer] = False

	"""
	ROBOT POSITION CONTROL
	"""
	def centre_on_gps_callback(self,event) -> None:
		# receives data from javascript on lnglat boundss
		eastBound = event['_northEast']['lng']
		westBound = event['_southWest']['lng']
		northBound = event['_northEast']['lat']
		southBound = event['_southWest']['lat']
		if not self.last_drawn_robot_position:
			return
		latitude = self.last_drawn_robot_position[0]
		longitude = self.last_drawn_robot_position[1]
		if latitude < southBound or \
			latitude > northBound or \
			longitude < westBound or \
			longitude > eastBound:
			self.map.panTo( [latitude,longitude]) #moves to new centre point
		
	def center_on_gps(self, gps_point) -> None:
		#pan to new location on map when robot reeaches edge of map
		#calls on getBounds, which requires callback function, where centering occurs
		self.map.getBounds(self.centre_on_gps_callback)

	def set_robot_position(self, lat: float, long: float) -> None:
		# if it's the first robot position, move the screen to it
		if not self.last_moved_robot_position:
			self.map.panTo([lat, long])

		# if we've moved a bit, redraw the robot
		if dist((lat, long), self.last_moved_robot_position) > self.ROBOT_REDRAW_THRESHOLD:
			self.robot.setLatLng([lat, long])
			self.last_moved_robot_position = (lat, long)
		# if we've moved more, add to the lines
		if dist((lat, long), self.last_drawn_robot_position) > self.LINE_DRAW_THRESHOLD:
			jscode = f'{self.robot_line.jsName}.addLatLng([{lat}, {long}]);'
			self.map.runJavaScript(jscode, 0)
			self.last_drawn_robot_position = [lat, long]

			for layer, points in self.map_points.items():
				self.draw_line_through_points(layer, points)

	def set_robot_rotation(self, angle: float) -> None:
		angle = angle + 90
		self.robot.setRotationAngle((angle) % 360)

	"""
	LINES
	"""
	def clear_lines(self):
		# Clear all layer lines
		for layer, points in self.map_points.items():
			# Use JavaScript to clear the polyline points
			self.map.runJavaScript(f'{self.points_line[layer].jsName}.setLatLngs([]);', 0)
			self.points_layer[layer].clearLayers()
		
		# Clear robot path line
		self.map.runJavaScript(f'{self.robot_line.jsName}.setLatLngs([]);', 0)
		
		# Get current robot position
		current_pos = None
		if self.last_moved_robot_position:
			current_pos = [self.last_moved_robot_position[0], self.last_moved_robot_position[1]]
			
		# Reset the last drawn position to current position
		self.last_drawn_robot_position = current_pos
		
		# Force update the map to clear all visible lines
		self.map.runJavaScript(f'{self.robot_line.jsName}.redraw();', 0)

	"""
	ELEVATION TODO- FINISH
	"""
	def display_elevation():
		# Needs to display the elevation map on top of the current map
		# maybe apply some colour filter over the map... may cause an issue bc it'll be dark tho 

		# set the ranges of elevation (should this change as we zoom?)
	

if __name__ == "__main__":
	import sys

	app = QApplication(sys.argv)
	viewer = MapViewer()
	
	viewer.show()  # Display the widget
	viewer.set_map_server(
		str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
	)
	viewer.set_robot_position(38.4,-110.78)
	
	viewer.set_robot_position(38.5,-110.78)

	
	
	sys.exit(app.exec_())