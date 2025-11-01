"""
MapViewer - contains methods to initialize and perform actions 
on a pyqtlet map 

The marker widget shows a collection of Marker objects onscreen 
relative to the robot's position.
"""

import json
import math

import os
from dataclasses import dataclass
from pathlib import Path

import uuid
import shutil
import tempfile


import map_js_snnipets

import csv
from datetime import datetime

from PyQt5 import QtWebChannel, QtCore

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

class JsBridge(QtCore.QObject):
	pointChosen = QtCore.pyqtSignal(float, float, str)           # lat, lng, label
	removePointSig = QtCore.pyqtSignal(str, float, float, str)   # id, lat, lng, label

	@QtCore.pyqtSlot(float, float, str)
	def storePoint(self, lat, lng, label):
		self.pointChosen.emit(lat, lng, label)

	@QtCore.pyqtSlot(str, float, float, str)
	def removePoint(self, point_id, lat, lng, label):
		self.removePointSig.emit(point_id, lat, lng, label)

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

	ELEVATION_GRID_PRECISION = 0.001

	headingSignal = pyqtSignal(float)

	def __init__(self, *args, **kwargs) -> None:
		super().__init__(*args, **kwargs)

		self._init_map_widget()
		self._init_map_config()
		self._init_markers(Locations.engineering)


		page = self._get_page()
		wc = getattr(page, 'webChannel', None)
		self._channel = (ch := (wc() if callable(wc) else wc)) or QtWebChannel.QWebChannel(page)
		if ch is None:
			page.setWebChannel(self._channel)
		if not hasattr(self, '_bridge'):
			self._bridge = JsBridge()
			self._channel.registerObject('pyBridge', self._bridge)

			# --- three fresh CSVs per app launch (with id column) ---
			self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
			base = Path.cwd()

			self.csv_perm = base / f'permanent_obstacles_{self.session_id}.csv'
			self.csv_temp = base / f'temporary_obstacles_{self.session_id}.csv'
			self.csv_trav = base / f'points_to_traverse_{self.session_id}.csv'

			for p in (self.csv_perm, self.csv_temp, self.csv_trav):
				with open(p, 'w', newline='') as f:
					writer = csv.DictWriter(f, fieldnames=['id','timestamp','lat','lng','label'])
					writer.writeheader()

			# index of live points: id -> (kind, csv_path)
			self.point_index = {}

			# connect signals
			self._bridge.pointChosen.connect(self._on_store_point)
			self._bridge.removePointSig.connect(self._on_remove_point)

			# (Optional) print paths for sanity
			print("Permanent obstacles CSV:", self.csv_perm)
			print("Temporary obstacles CSV:", self.csv_temp)
			print("Points to traverse CSV:", self.csv_trav)
		self._install_right_click_popup()


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

		# initialize elevation data
		self.elevation_data: dict = {}
		self.elevation_layer = None
		self.elevation_min: float = 0
		self.elevation_max: float = 0

	"""
	INIT METHODS
	"""
	def _get_page(self):
    # Prefer MapWidget.view.page if present
		if hasattr(self, 'mapWidget'):
			view = getattr(self.mapWidget, 'view', None)
			if view is not None:
				p = getattr(view, 'page', None)
				if p is not None:
					return p() if callable(p) else p

			# Some builds expose .page directly on MapWidget
			p = getattr(self.mapWidget, 'page', None)
			if p is not None:
				return p() if callable(p) else p

		# Fallback: pyqtlet2 Map may expose a view with a page
		if hasattr(self, 'map') and hasattr(self.map, 'view'):
			p = getattr(self.map.view, 'page', None)
			if p is not None:
				return p() if callable(p) else p

		raise RuntimeError("Could not obtain QWebEnginePage for WebChannel.")

	# --- INTERNAL: handle a stored point from JS (append CSV + draw a marker) ---
	def _on_store_point(self, lat: float, lng: float, label: str):
		key = (label or '').strip().lower()
		if key == 'permanent_obstacle':
			target = self.csv_perm; color = '#d62728'; tip_label = 'Permanent obstacle'; kind = 'perm'
		elif key == 'temporary_obstacle':
			target = self.csv_temp; color = '#ff7f0e'; tip_label = 'Temporary obstacle'; kind = 'temp'
		elif key == 'point_to_traverse':
			target = self.csv_trav; color = '#1f77b4'; tip_label = 'Point to traverse'; kind = 'trav'
		else:
			target = self.csv_temp; color = '#7f7f7f'; tip_label = f'Unknown: {label}'; kind = 'temp'

		# Assign a unique id and persist it
		pid = str(uuid.uuid4())
		with open(target, 'a', newline='') as f:
			writer = csv.DictWriter(f, fieldnames=['id','timestamp','lat','lng','label'])
			writer.writerow({
				'id': pid,
				'timestamp': datetime.now().isoformat(timespec='seconds'),
				'lat': f'{lat:.8f}',
				'lng': f'{lng:.8f}',
				'label': key
			})
		self.point_index[pid] = (kind, str(target))

		# Draw marker; MIDDLE-CLICK to remove this point
		js = f"""
		(function() {{
		var m = window['{self.map.jsName}Object'] || window.map;
		if (!m) {{ console.error('Leaflet map not found'); return; }}

		function ensureBridge(ready) {{
			if (window.pyBridge) {{ ready(); return; }}
			function start() {{
			if (window.QWebChannel) {{
				new QWebChannel(qt.webChannelTransport, function(channel) {{
				window.pyBridge = channel.objects.pyBridge; ready();
				}});
			}} else {{
				var s = document.createElement('script');
				s.src = 'qrc:///qtwebchannel/qwebchannel.js';
				s.onload = function() {{ start(); }};
				document.head.appendChild(s);
			}}
			}} start();
		}}

		window._storedGroup = window._storedGroup || L.layerGroup().addTo(m);
		var mk = L.circleMarker([{lat}, {lng}], {{radius:6, color:'{color}', weight:2}});
		mk.bindTooltip('{tip_label} ({lat:.5f}, {lng:.5f})');
		mk.options.pid = '{pid}';

		// Middle-click removal: mouse button 1
		mk.on('mousedown', function(e) {{
			var oe = e.originalEvent || e;  // DOM MouseEvent
			if (oe && oe.button === 1) {{
			if (L && L.DomEvent) {{ L.DomEvent.preventDefault(oe); L.DomEvent.stop(oe); }}
			ensureBridge(function() {{
				if (window.pyBridge && window.pyBridge.removePoint) {{
				window.pyBridge.removePoint('{pid}', {lat}, {lng}, '{key}');
				}}
			}});
			m.removeLayer(mk);
			}}
		}});

		mk.addTo(window._storedGroup);
		}})();
		"""
		self._get_page().runJavaScript(js)

	def _on_remove_point(self, pid: str):
		"""
		Remove a point from its CSV. Primary key is 'id'. If the CSV has no 'id' column
		(legacy rows), fall back to matching by lat/lng/label (string-formatted).
		"""
		info = self.point_index.get(pid)

		# Try the known file first; otherwise scan all three
		candidate_paths = [info[1]] if info else [str(self.csv_perm), str(self.csv_temp), str(self.csv_trav)]

		removed = False
		lat_s = f"{lat:.8f}"
		lng_s = f"{lng:.8f}"
		label_s = (label or "").strip().lower()

		for path in candidate_paths:
			if not os.path.exists(path):
				continue

			tmp = tempfile.NamedTemporaryFile('w', delete=False, newline='')
			try:
				with open(path, 'r', newline='') as src, tmp as out:
					r = csv.DictReader(src)
					# Handle legacy files without 'id'
					fieldnames = r.fieldnames or ['timestamp','lat','lng','label']
					if 'id' not in fieldnames:
						fieldnames = ['id','timestamp','lat','lng','label']

					w = csv.DictWriter(out, fieldnames=fieldnames)
					w.writeheader()

					for row in r:
						row_id = row.get('id')
						row_lat = row.get('lat')
						row_lng = row.get('lng')
						row_label = (row.get('label') or '').strip().lower()

						match_by_id = (row_id is not None and row_id == pid)
						match_by_vals = (row_id in (None, '',)) and (row_lat == lat_s and row_lng == lng_s and row_label == label_s)

						if match_by_id or match_by_vals:
							removed = True
							continue  # skip (delete)
						else:
							# Ensure legacy rows gain an 'id' column on rewrite
							if 'id' in fieldnames and 'id' not in row:
								row['id'] = row_id or ''
							w.writerow(row)
				shutil.move(out.name, path)
			finally:
				try:
					os.unlink(tmp.name)
				except Exception:
					pass

			if removed:
				break

		# Drop from in-memory index if we knew it
		if pid in self.point_index:
			self.point_index.pop(pid, None)

	# --- INTERNAL: inject right-click popup JS with 2 buttons calling back to Python ---
	def _install_right_click_popup(self):
		js = f"""
		(function() {{
		var m = window['{self.map.jsName}Object'] || window.map;
		if (!m) {{ console.error('Leaflet map not found'); return; }}

		function ensureBridge(ready) {{
			if (window.pyBridge) {{ ready(); return; }}
			function start() {{
			if (window.QWebChannel) {{
				new QWebChannel(qt.webChannelTransport, function(channel) {{
				window.pyBridge = channel.objects.pyBridge;
				ready();
				}});
			}} else {{
				var s = document.createElement('script');
				s.src = 'qrc:///qtwebchannel/qwebchannel.js';
				s.onload = function() {{ start(); }};
				document.head.appendChild(s);
			}}
			}}
			start();
		}}

		function buildPopup(lat, lng) {{
			var latStr = lat.toFixed(6), lngStr = lng.toFixed(6);
			return ''
			+ '<div style="min-width:220px">'
			+ '  <div style="font-weight:600; margin-bottom:6px">Store this location as...</div>'
			+ '  <div style="font-size:12px; opacity:.8; margin-bottom:8px">(' + latStr + ', ' + lngStr + ')</div>'
			+ '  <div style="display:flex; gap:6px; flex-direction:column">'
			+ '    <button id="store-perm" style="padding:6px 8px">Permanent obstacle</button>'
			+ '    <button id="store-temp" style="padding:6px 8px">Temporary obstacle</button>'
			+ '    <button id="store-trav" style="padding:6px 8px">Point to traverse</button>'
			+ '  </div>'
			+ '</div>';
		}}

		m.on('contextmenu', function(e) {{
			var lat = e.latlng.lat, lng = e.latlng.lng;
			var popup = L.popup({{ closeOnClick: true }})
			.setLatLng(e.latlng)
			.setContent(buildPopup(lat, lng))
			.openOn(m);

			setTimeout(function() {{
			function send(label) {{
				ensureBridge(function() {{
				if (window.pyBridge && window.pyBridge.storePoint) {{
					window.pyBridge.storePoint(lat, lng, label);
				}} else {{
					console.error('pyBridge not ready');
				}}
				}});
				m.closePopup(popup);
			}}
			var btnPerm = document.getElementById('store-perm');
			var btnTemp = document.getElementById('store-temp');
			var btnTrav = document.getElementById('store-trav');
			if (btnPerm) btnPerm.addEventListener('click', function(){{ send('permanent_obstacle'); }});
			if (btnTemp) btnTemp.addEventListener('click', function(){{ send('temporary_obstacle'); }});
			if (btnTrav) btnTrav.addEventListener('click', function(){{ send('point_to_traverse'); }});
			}}, 0);
		}});
		}})();
		"""
		# Run on the raw WebEngine page (not via pyqtlet2 Map)
		self._get_page().runJavaScript(js)


	
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
			map_js_snnipets.map_marker_set_lat_lng_points(self.map, [], self.points_line[layer].jsName)
			self.points_layer[layer].clearLayers()
		
		# Clear robot path line
		map_js_snnipets.map_marker_set_lat_lng_points(self.map, [], self.robot_line.jsName)
		
		# Get current robot position
		current_pos = None
		if self.last_moved_robot_position:
			current_pos = [self.last_moved_robot_position[0], self.last_moved_robot_position[1]]
			
		# Reset the last drawn position to current position
		self.last_drawn_robot_position = current_pos
		
		# Force update the map to clear all visible lines
		map_js_snnipets.map_line_redraw(self.map, self.robot_line.jsName)

	"""
	ELEVATION
	"""
	def set_elevation_source(self, file_path: str) -> None:
		"""Load elevation data from a JSON file."""
		try:
			with open(file_path, 'r') as f:
				raw_data = json.load(f)
			
			self.elevation_data = {}
			elevations = []
			
			for coord_key, elevation in raw_data.items():
				lat_str, long_str = coord_key.split(',')
				self.elevation_data[(float(lat_str), float(long_str))] = float(elevation)
				elevations.append(elevation)
			
			# Calculate min and max elevation values for color mapping
			if elevations:
				self.elevation_min = min(elevations)
				self.elevation_max = max(elevations)
				print(f"Elevation data loaded: {len(elevations)} points")
				print(f"Elevation range: {self.elevation_min:.2f} to {self.elevation_max:.2f}")
		
		except FileNotFoundError:
			print(f"Error: Elevation file not found, run elevation_downloader to get some data: {file_path}")
		except json.JSONDecodeError:
			print(f"Error: Invalid JSON format in {file_path}")
		except Exception as e:
			print(f"Error loading elevation data: {e}")

	def _elevation_to_color(self, elevation: float, color: tuple[int,int,int] = (255, 0, 0)) -> str:
		"""Convert elevation value to a color (darker (low) ---> brighter (high)). Return the hex value."""
		r, g, b = color
		if self.elevation_max == self.elevation_min:
			return f"#{r:02x}{g:02x}{b:02x}"
		
		# Normalize elevation to 0-1 range
		normalized = (elevation - self.elevation_min) / (self.elevation_max - self.elevation_min)
		
		r_out = int(normalized * r)
		g_out = int(normalized * g)
		b_out = int(normalized * b)
		
		return f"#{r_out:02x}{g_out:02x}{b_out:02x}"

	def display_elevation(self) -> None:
		"""
		Start elevation data as colored dots on the map.
		Iterates through all coordinates in the current viewport with a interval
		and draws dots colored by elevation
		Will refresh on move and zoom.
		"""
		if not self.elevation_data:
			return
		
		# Remove existing elevation layer if present
		if self.elevation_layer:
			self.map.removeLayer(self.elevation_layer)
		
		# Create new layer group for elevation dots
		self.elevation_layer = L.layerGroup()
		self.elevation_layer.addTo(self.map)
		
		# Get current map bounds
		def process_bounds(bounds):
			south = bounds['_southWest']['lat']
			north = bounds['_northEast']['lat']
			west = bounds['_southWest']['lng']
			east = bounds['_northEast']['lng']
			
			print(f"Rendering elevation for bounds:")
			print(f"  Latitude: {south:.4f} to {north:.4f}")
			print(f"  Longitude: {west:.4f} to {east:.4f}")
			
			dot_count = 0
			
			# Iterate through coordinates 
			lat = south
			while lat <= north:
				long = west
				while long <= east:
					decimals = -int(math.log10(self.ELEVATION_GRID_PRECISION))
					coord_key = (round(lat, decimals), round(long, decimals))
					
					# Check if we have elevation data for this coordinate
					if coord_key in self.elevation_data:
						elevation = self.elevation_data[coord_key]
						color = self._elevation_to_color(elevation)
						
						circlemarker = L.circleMarker(list(coord_key), {
							"radius": 10,
							"fillColor": color,
							"color": color,
							"weight": 3,
						})
						# need to add before bind popup
						circlemarker.addTo(self.elevation_layer)
						circlemarker.bindPopup(elevation)
						dot_count += 1
					long += self.ELEVATION_GRID_PRECISION
				lat += self.ELEVATION_GRID_PRECISION
			
			print(f"Rendered {dot_count} dots")
		
		# Get bounds asynchronously and process them
		self.map.getBounds(process_bounds)

if __name__ == "__main__":
	import sys

	app = QApplication(sys.argv)
	viewer = MapViewer()
	
	viewer.show()  # Display the widget
	viewer.set_map_server(
		str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
	)
	viewer.set_robot_position(43.658,-79.398)

	viewer.set_elevation_source(resolve_path("elevation.json"))
	from PyQt5.QtCore import QTimer
	def show_elevation():
		print("Displaying elevation data...")
		viewer.display_elevation()
	
	QTimer.singleShot(1000, show_elevation)

	viewer.map.zoom.connect(show_elevation)
	
	# viewer.set_robot_position(38.5,-110.78)

	
	
	sys.exit(app.exec_())