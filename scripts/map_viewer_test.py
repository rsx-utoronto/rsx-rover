#!/usr/bin/env python3.8

"""The marker widget shows a collection of Marker objects onscreen relative to the robot's position."""

import math

import os
from enum import Enum
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict

os.environ['QT_API'] = 'pyqt5'
import pyqtlet2
from pyqtlet2 import MapWidget
from pyqtlet2.leaflet import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"

@dataclass
class MapPoint:
	latitude: float
	longitude: float
	radius: float
	name: str

class Locations(Enum):
	engineering = [(38.380691+38.439607)/2,(-110.829048+-110.751286)/2]
	mdrs = [38.406458, -110.791903]

	def __str__(self):
		return str(self.value)

	def __getitem__(self, item):
		return self.value[item]

class MapViewer(QWidget):
	"""View markers and robot position overlaid on a satellite map."""

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.mapWidget = MapWidget()
		self.layout = QVBoxLayout()
		self.layout.setContentsMargins(5, 0, 0, 0)
		self.layout.addWidget(self.mapWidget)
		self.setLayout(self.layout)

		# create pyqtlet2 map
		self.map = L.map(
			self.mapWidget,
			{
				"doubleClickZoom": False,
				"maxBountsViscosity": 1.0,
			}
		)

		# initialize view and cursor
		self.map.setView(Locations.mdrs, 18)
		self.map.runJavaScript(
		"""
		var styleSheet = document.createElement("style")
		styleSheet.type = "text/css"
		styleSheet.innerText = ".leaflet-container { cursor:crosshair; }"
		document.head.appendChild(styleSheet)
		"""
		, 0)

		# 'initialize' tile layer
		self.tile_layer = None


		# initialize robot icon
		robot_startup_location = Locations.engineering.value
		self.robot_icon = L.icon(str(Path(__file__).parent.resolve() / "../resources/robot.png"),
		{
			"iconSize": [26, 25],
			"iconAnchor": [10, 12]
		})
		self.robot = L.marker(robot_startup_location, {
			'rotationAngle': 0,
			'rotationOrigin': '10px 12px'
		})
		self.robot.setIcon(self.robot_icon)
		self.robot.addTo(self.map)

		# initialize robot line
		self.last_moved_robot_position = None
		self.last_drawn_robot_position = robot_startup_location
		self.robot_line = L.polyline([], {'color': 'red'})
		self.robot_line.addTo(self.map)

		# initialize dictionaries for storing info
		self.map_points: Dict[str, List[MapPoint]] = {}
		self.points_line: Dict[str, Polyline] = {}
		self.points_layer: Dict[str, LayerGroup] = {}
		self.selected_marker: Dict[str, Circle] = {}
		self.show_selected_marker: Dict[str, bool] = {}
		self.circle_color: Dict[str, str] = {}

	def set_map_server(self, tile_url: str, layer_count):
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

	def add_popup_marker(self, location, popup_message):
		popup_marker = L.marker(location)
		popup_marker.bindPopup(popup_message)
		popup_marker.addTo(self.map)

	def add_point_layer(self, layer_name: str, circle_color: str, line_color: str, selected_color: str):
		self.points_line[layer_name] = Polyline([], {'color': line_color})
		self.points_layer[layer_name] = LayerGroup()
		self.selected_marker[layer_name] = Circle([0, 0], 2, {'color': selected_color, 'weight': 8})
		self.show_selected_marker[layer_name] = False
		self.circle_color[layer_name] = circle_color

		self.points_line[layer_name].addTo(self.map)
		self.points_layer[layer_name].addTo(self.map)

	def draw_line_through_points(self, layer: str, points: List[MapPoint]):
		markers_line = self.points_line[layer]
		latlngs = [self.last_drawn_robot_position] + [[p.latitude, p.longitude] for p in points]
		self.map.runJavaScript(f'{markers_line.jsName}.setLatLngs({latlngs});', 1)

	def set_points(self, layer: str, points: List[MapPoint]):
		self.map_points[layer] = points

		# draw the new circles
		self.points_layer[layer].clearLayers()
		for point in points:
			c = Circle([point.latitude, point.longitude], point.radius, {'weight': 4})
			if point.name:
				_bindTooltip(c, point.name, {'permanent': True, 'direction': 'down'})
			self.points_layer[layer].addLayer(c)

		self.draw_line_through_points(layer, points)

	def set_highlighted_point(self, layer: str, index: int):
		lat = self.map_points[layer][index].latitude
		long = self.map_points[layer][index].longitude
		radius = self.map_points[layer][index].radius

		self.map.runJavaScript(f'{self.selected_marker[layer].jsName}.setLatLng([{lat}, {long}]);', 0)
		self.map.runJavaScript(f'{self.selected_marker[layer].jsName}.setRadius({radius + 2});', 0)
		if not self.show_selected_marker[layer]:
			self.selected_marker[layer].addTo(self.map)
		self.show_selected_marker[layer] = True
	
	def unhighlight_point(self, layer: str):
		if self.show_selected_marker[layer]:
			self.selected_marker[layer].removeFrom(self.map)
			self.show_selected_marker[layer] = False

	def set_robot_position(self, lat: float, long: float):
		# if it's the first robot position, move the screen to it
		if not self.last_moved_robot_position:
			self.map.panTo([lat, long])

		# if we've moved a bit, redraw the robot
		if dist((lat, long), self.last_moved_robot_position) > 0.000001:
			self.robot.setLatLng([lat, long])
			self.last_moved_robot_position = (lat, long)

		# if we've moved more, add to the lines
		if dist((lat, long), self.last_drawn_robot_position) > 0.00001:
			jscode = f'{self.robot_line.jsName}.addLatLng([{lat}, {long}]);'
			self.map.runJavaScript(jscode, 0)
			self.last_drawn_robot_position = [lat, long]

			for layer, points in self.map_points.items():
				self.draw_line_through_points(layer, points)

	def set_robot_rotation(self, angle: float):
		# angle is in degrees (0 - 360). 0 is right
		self.robot.setRotationAngle((angle) % 360)

def _bindTooltip(layer: pyqtlet2.leaflet.layer.Layer, content: str, options=None):
	js = '{layerName}.bindTooltip("{content}"'.format(
			layerName=layer._layerName, content=content)
	if options:
		js += ', {options}'.format(options=layer._stringifyForJs(options))
	js += ')'
	layer.runJavaScript(js, 0)
	return layer

def dist(latlong1, latlong2):
	if not latlong1 or not latlong2:
		return math.inf

	return math.sqrt((latlong1[0] - latlong2[0])**2 + (latlong1[1] - latlong2[1])**2)

if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    viewer = MapViewer()
	# initial_map_server = MapServers.ARCGIS_World_Imagery_Cache
    viewer.set_map_server(str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg',20)  # Example tile server
    viewer.show()  # Display the widget
    sys.exit(app.exec_())