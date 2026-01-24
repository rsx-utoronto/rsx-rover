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
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication, QPushButton, QMessageBox

import uuid
from collections import defaultdict

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
    # existing
    pointChosen = QtCore.pyqtSignal(float, float, str)                 # lat, lng, label
    removePointSig = QtCore.pyqtSignal(str, float, float, str)         # pid, lat, lng, label

    selectObstacleSig = QtCore.pyqtSignal(str, str)                    # obstacle_id, category ('temporary'|'permanent')
    removeObstacleSig = QtCore.pyqtSignal(str, str)                    # obstacle_id, category

    @QtCore.pyqtSlot(float, float, str)
    def storePoint(self, lat, lng, label):
        self.pointChosen.emit(lat, lng, label)

    @QtCore.pyqtSlot(str, float, float, str)
    def removePoint(self, point_id, lat, lng, label):
        self.removePointSig.emit(point_id, lat, lng, label)

    @QtCore.pyqtSlot(str, str)
    def selectObstacle(self, obstacle_id, category):
        self.selectObstacleSig.emit(obstacle_id, category)

    @QtCore.pyqtSlot(str, str)
    def removeObstacle(self, obstacle_id, category):
        self.removeObstacleSig.emit(obstacle_id, category)

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
	OBSTACLE_CLEARANCE = 0.000002
	
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
			self.temp_vertices = []   # list of (lat, lng) for Temporary obstacle
			self.perm_vertices = []   # list of (lat, lng) for Permanent obstacle

			# --- pick CSV files: reuse latest ones if they exist, otherwise create new ones ---
			self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
			base = Path.cwd()

			def _choose_csv(pattern, default_name, fieldnames):
				"""
                If there is an existing CSV matching `pattern` (e.g.
                'temporary_obstacles_*.csv'), reuse the newest one.
                Otherwise create a new file called `default_name` with the
                given header.
                """
				matches = sorted(base.glob(pattern))
				if matches:
                    # Reuse the newest existing file
					return matches[-1]

                # No file yet -> create a new one with header
				path = base / default_name
				with open(path, 'w', newline='') as f:
					writer = csv.DictWriter(f, fieldnames=fieldnames)
					writer.writeheader()
				return path

			self.csv_perm = _choose_csv(
                'permanent_obstacles_*.csv',
                f'permanent_obstacles_{self.session_id}.csv',
                ['id', 'timestamp', 'lat', 'lng', 'label', 'obstacle_id'],
            )
			self.csv_temp = _choose_csv(
                'temporary_obstacles_*.csv',
                f'temporary_obstacles_{self.session_id}.csv',
                ['id', 'timestamp', 'lat', 'lng', 'label', 'obstacle_id'],
            )
			self.csv_trav = _choose_csv(
                'points_to_traverse_*.csv',
                f'points_to_traverse_{self.session_id}.csv',
                ['id', 'timestamp', 'lat', 'lng', 'label'],
            )

			# CSV to store path points for straight-line segments
			base = Path.cwd()
			self.csv_path_segments = base / f'path_segments_{self.session_id}.csv'
			with open(self.csv_path_segments, 'w', newline='') as f:
				writer = csv.DictWriter(f, fieldnames=['id', 'timestamp', 'lat', 'lng', 'label'])
				writer.writeheader()

			# index of live points: id -> (kind, csv_path)
			self.point_index = {}

			# connect signals
			self._bridge.pointChosen.connect(self._on_store_point)
			self._bridge.removePointSig.connect(self._on_remove_point)

			self._bridge.selectObstacleSig.connect(self._on_select_obstacle)
			self._bridge.removeObstacleSig.connect(self._on_remove_obstacle)

			self.obstacles = {'temporary': defaultdict(list), 'permanent': defaultdict(list)}
			self.active_obstacle = {'temporary': None, 'permanent': None}
			self.selected_obstacle = {'temporary': None, 'permanent': None}
			self.point_to_obstacle = {}  # pid -> (category, obstacle_id)
            # per-category running counter; obstacle_id will just be this number as a string
			self.obstacle_counter = {'temporary': 0, 'permanent': 0}

			# (Optional) print paths for sanity
			print("Permanent obstacles CSV:", self.csv_perm)
			print("Temporary obstacles CSV:", self.csv_temp)
			print("Points to traverse CSV:", self.csv_trav)
		self._install_right_click_popup()
		self._install_mouse_position_display()


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
		self.places_to_go_markers = []

		# initialize arm radius stuff 
		self.arm_layer = None


	def _install_mouse_position_display(self):
		js = f"""
		(function() {{
			var m = window['{self.map.jsName}Object'] || window.map;
			if (!m) {{ console.error('Leaflet map not found'); return; }}

			// Avoid initializing twice
			if (m._mouseCoordControlInitialized) return;
			m._mouseCoordControlInitialized = true;

			// Create a small Leaflet control in the bottom-left
			var coordsControl = L.control({{position: 'bottomleft'}});
			coordsControl.onAdd = function(map) {{
				var div = L.DomUtil.create('div', 'leaflet-control-mousecoords');
				div.style.background = 'rgba(255,255,255,0.8)';
				div.style.padding = '2px 6px';
				div.style.margin = '0 0 3px 0';
				div.style.fontSize = '11px';
				div.style.fontFamily = 'monospace';
				div.innerHTML = 'Lat: -- , Lng: --';
				return div;
			}};
			coordsControl.addTo(m);

			// Update on mouse move
			m.on('mousemove', function(e) {{
				var lat = e.latlng.lat.toFixed(6);
				var lng = e.latlng.lng.toFixed(6);
				var els = document.getElementsByClassName('leaflet-control-mousecoords');
				if (els && els[0]) {{
					els[0].innerHTML = 'Lat: ' + lat + ' , Lng: ' + lng;
				}}
			}});

			// Clear when mouse leaves the map
			m.on('mouseout', function(e) {{
				var els = document.getElementsByClassName('leaflet-control-mousecoords');
				if (els && els[0]) {{
					els[0].innerHTML = 'Lat: -- , Lng: --';
				}}
			}});
		}})();
		"""
		self._get_page().runJavaScript(js)

	def _collect_obstacle_hulls_for_path(self):
		"""
		Build convex hulls for all obstacles from self.obstacles so that we
		can test segments against them.
		Returns: list of {'cat': ..., 'oid': ..., 'verts': [(lat,lng), ...]}
		"""
		hulls = []
		for cat in ("permanent", "temporary"):
			for oid, pts in self.obstacles[cat].items():
				raw = [(p["lat"], p["lng"]) for p in pts]
				raw = sorted(set(raw))
				if len(raw) < 2:
					continue

				if len(raw) == 2:
					hull = raw
				else:
					def cross(o, a, b):
						return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

					lower = []
					for p in raw:
						while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
							lower.pop()
						lower.append(p)

					upper = []
					for p in reversed(raw):
						while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
							upper.pop()
						upper.append(p)

					hull = lower[:-1] + upper[:-1]

				hulls.append({"cat": cat, "oid": oid, "verts": hull})
		return hulls

	def _segments_intersect(self, p1, p2, q1, q2):
		"""Standard segment intersection test."""
		def orient(a, b, c):
			return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

		def on_seg(a, b, c):
			return (min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and
			        min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))

		o1 = orient(p1, p2, q1)
		o2 = orient(p1, p2, q2)
		o3 = orient(q1, q2, p1)
		o4 = orient(q1, q2, p2)

		# general case
		if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
			return True

		# colinear special cases
		if o1 == 0 and on_seg(p1, p2, q1):
			return True
		if o2 == 0 and on_seg(p1, p2, q2):
			return True
		if o3 == 0 and on_seg(q1, q2, p1):
			return True
		if o4 == 0 and on_seg(q1, q2, p2):
			return True

		return False

	def _dist_point_to_segment(self, p, a, b):
		"""Euclidean distance (in degrees) from point p to segment ab."""
		if not a or not b:
			return math.inf
		if a == b:
			return dist(p, a)

		ax, ay = a
		bx, by = b
		px, py = p

		vx = bx - ax
		vy = by - ay
		wx = px - ax
		wy = py - ay

		den = vx * vx + vy * vy
		if den == 0:
			return dist(p, a)

		t = (vx * wx + vy * wy) / den
		if t < 0.0:
			closest = (ax, ay)
		elif t > 1.0:
			closest = (bx, by)
		else:
			closest = (ax + t * vx, ay + t * vy)

		return dist(p, closest)
	
	def _segment_blocked_by_obstacles(self, p1, p2, obstacle_edges, tol=1e-9):
		"""
		Return True if segment p1-p2 crosses any obstacle edge (other than
		sharing endpoints exactly).
		"""
		def almost_equal(a, b, eps=tol):
			return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps

		for (c, d) in obstacle_edges:
			# allow touching at shared endpoints
			if (almost_equal(p1, c) or almost_equal(p1, d) or
			    almost_equal(p2, c) or almost_equal(p2, d)):
				continue
			if self._segments_intersect(p1, p2, c, d):
				return True
		return False

	def _dijkstra_path(self, adj, start_idx, goal_idx):
		"""Simple Dijkstra shortest path."""
		import heapq

		N = len(adj)
		distances = [math.inf] * N
		prev = [None] * N
		distances[start_idx] = 0.0
		h = [(0.0, start_idx)]

		while h:
			d, u = heapq.heappop(h)
			if d > distances[u]:
				continue
			if u == goal_idx:
				break
			for v, w in adj[u]:
				nd = d + w
				if nd < distances[v]:
					distances[v] = nd
					prev[v] = u
					heapq.heappush(h, (nd, v))

		if distances[goal_idx] == math.inf:
			return None

		path = []
		cur = goal_idx
		while cur is not None:
			path.append(cur)
			cur = prev[cur]
		path.reverse()
		return path

	def _plan_path_between_points(self, start, goal, hulls):
		"""
		Visibility-graph style path between start and goal that avoids
		obstacle polygons (built from hulls).
		Returns: list of (lat, lng) including start and goal.
		"""
		# nodes[0] = start, nodes[1] = goal, others = obstacle vertices
		nodes = [start, goal]
		meta = [None, None]  # (cat, oid, idx, nverts) for obstacle vertices

		for ob in hulls:
			verts = ob["verts"]
			n = len(verts)
			for idx, (lat, lng) in enumerate(verts):
				nodes.append((lat, lng))
				meta.append((ob["cat"], ob["oid"], idx, n))

		# build list of all obstacle edges for intersection tests
		obstacle_edges = []
		for ob in hulls:
			verts = ob["verts"]
			m = len(verts)
			if m < 2:
				continue
			if m == 2:
				obstacle_edges.append((verts[0], verts[1]))
			else:
				for i in range(m):
					obstacle_edges.append((verts[i], verts[(i + 1) % m]))

		N = len(nodes)
		adj = [[] for _ in range(N)]

		for i in range(N):
			for j in range(i + 1, N):
				p1 = nodes[i]
				p2 = nodes[j]
				if dist(p1, p2) == 0:
					continue

				mi = meta[i]
				mj = meta[j]

				# if both are vertices of the same obstacle, only connect neighbours on the hull
				if mi is not None and mj is not None and mi[0] == mj[0] and mi[1] == mj[1]:
					n = mi[3]
					idx_i = mi[2]
					idx_j = mj[2]
					if not ((idx_i - idx_j) % n == 1 or (idx_j - idx_i) % n == 1):
						# non-neighbour vertices -> would cut across polygon interior
						continue

				if self._segment_blocked_by_obstacles(p1, p2, obstacle_edges):
					continue

				w = dist(p1, p2)
				adj[i].append((j, w))
				adj[j].append((i, w))

		path_idx = self._dijkstra_path(adj, 0, 1)
		if not path_idx:
			# if graph failed but direct line is free, just use it
			if not self._segment_blocked_by_obstacles(start, goal, obstacle_edges):
				return [start, goal]
			else:
				# last-resort fallback: still return direct
				return [start, goal]

		return [nodes[k] for k in path_idx]
	
	def make_straight_line_path(self):
		"""
		Build a straight-line path that:
		  - starts at 'Start' (if present in places_to_go),
		  - always goes to the nearest remaining point,
		  - and between each pair uses straight-line segments that avoid obstacles.

		The resulting path is drawn on the map and its points (without duplicates)
		are stored in self.csv_path_segments.
		"""
		pts = getattr(self, "places_to_go_points", None)
		if not pts:
			QMessageBox.warning(
				self,
				"Path",
				"No 'places to go' points loaded.\n\nClick 'Load Places To Go' first."
			)
			return

		if len(pts) < 2:
			QMessageBox.information(
				self,
				"Path",
				"Need at least two points to build a path."
			)
			return

		# pick start point: row named "Start" if present, else first point
		start_idx = None
		for i, p in enumerate(pts):
			if (p.get("name") or "").strip().lower() == "start":
				start_idx = i
				break
		if start_idx is None:
			start_idx = 0

		def coord(idx):
			return (pts[idx]["lat"], pts[idx]["lng"])

		unvisited = set(range(len(pts)))
		unvisited.discard(start_idx)
		current_idx = start_idx

		# build obstacle hulls from loaded obstacles
		hulls = self._collect_obstacle_hulls_for_path()

		full_path = [coord(current_idx)]

		# greedy nearest-neighbour order
		while unvisited:
			cur_xy = coord(current_idx)
			next_idx = min(unvisited, key=lambda k: dist(cur_xy, coord(k)))
			next_xy = coord(next_idx)

			segment_path = self._plan_path_between_points(cur_xy, next_xy, hulls)

			# append, skipping first point to avoid duplicates between segments
			for (lat, lng) in segment_path[1:]:
				if dist((lat, lng), full_path[-1]) > 1e-9:
					full_path.append((lat, lng))

			current_idx = next_idx
			unvisited.remove(next_idx)

		# ----- draw path on map -----
		if not hasattr(self, "path_line"):
			self.path_line = L.polyline([], {"color": "#00ff00"})
			self.path_line.addTo(self.map)

		latlngs = [[lat, lng] for (lat, lng) in full_path]

		# clear previous path then draw new one
		map_js_snnipets.map_marker_set_lat_lng_points(self.map, [], self.path_line.jsName)
		map_js_snnipets.map_marker_set_lat_lng_points(self.map, latlngs, self.path_line.jsName)

		# ----- save unique points to CSV (no duplicates) -----
		seen = set()
		ordered_unique = []
		for (lat, lng) in full_path:
			key = (round(lat, 8), round(lng, 8))
			if key not in seen:
				seen.add(key)
				ordered_unique.append((lat, lng))

		with open(self.csv_path_segments, "w", newline="") as f:
			writer = csv.DictWriter(
				f,
				fieldnames=["id", "timestamp", "lat", "lng", "label"]
			)
			writer.writeheader()
			for (lat, lng) in ordered_unique:
				writer.writerow({
					"id": str(uuid.uuid4()),
					"timestamp": datetime.now().isoformat(timespec="seconds"),
					"lat": f"{lat:.8f}",
					"lng": f"{lng:.8f}",
					"label": "path_point",
				})

		QMessageBox.information(
			self,
			"Path",
			f"Path built with {len(full_path)} points.\n"
			f"Saved {len(ordered_unique)} unique points to:\n{self.csv_path_segments}"
		)
	
	def _redraw_obstacle_shape(self, cat: str, oid: str):
		import json

		cat = 'temporary' if cat.startswith('temporary') else 'permanent'
		pts = self.obstacles.get(cat, {}).get(oid, [])

		# If fewer than 2 points: clear any existing shape AND label and return
		if not pts or len(pts) < 2:
			js = f"""
			(function(){{
				var m = window['{self.map.jsName}Object'] || window.map;
				if (!m) return;
				window._obsShapes = window._obsShapes || {{temporary:{{}}, permanent:{{}}}};
				window._obsLabels = window._obsLabels || {{temporary:{{}}, permanent:{{}}}};
				if (window._obsShapes['{cat}'] && window._obsShapes['{cat}']['{oid}']) {{
					try {{ m.removeLayer(window._obsShapes['{cat}']['{oid}']); }} catch(e) {{}}
					window._obsShapes['{cat}']['{oid}'] = null;
				}}
				if (window._obsLabels['{cat}'] && window._obsLabels['{cat}']['{oid}']) {{
					try {{ m.removeLayer(window._obsLabels['{cat}']['{oid}']); }} catch(e) {{}}
					window._obsLabels['{cat}']['{oid}'] = null;
				}}
			}})();
			"""
			self._get_page().runJavaScript(js)
			return

		# Build convex hull of this obstacle's vertices
		raw = [(p['lat'], p['lng']) for p in pts]
		pts_unique = sorted(set(raw))
		if len(pts_unique) == 1:
			# still a single unique point -> nothing to draw yet
			return

		if len(pts_unique) == 2:
			hull = pts_unique
		else:
			def cross(o, a, b):
				return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

			lower = []
			for p in pts_unique:
				while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
					lower.pop()
				lower.append(p)

			upper = []
			for p in reversed(pts_unique):
				while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
					upper.pop()
				upper.append(p)

			hull = lower[:-1] + upper[:-1]

		# coords of hull for polygon/polyline
		coords = [[lat, lng] for (lat, lng) in hull]
		coords_json = json.dumps(coords)

		# center of hull (simple average)
		lat_center = sum(lat for (lat, _lng) in hull) / len(hull)
		lng_center = sum(lng for (_lat, lng) in hull) / len(hull)

		# label text for this obstacle – just use its obstacle_id (e.g., "1", "2", "3")
		label_text = str(oid)

		# Style based on selection
		is_selected = (self.selected_obstacle.get(cat) == oid)
		color = "#ff7f0e" if cat == "temporary" else "#d62728"
		weight = 3 if is_selected else 2
		dash = "null" if is_selected else "'6,4'"
		fill_opacity = "0.25" if len(coords) >= 3 else "0.0"

		js = f"""
		(function(){{
			var m = window['{self.map.jsName}Object'] || window.map;
			if (!m) {{ console.error('Leaflet map not found'); return; }}

			window._obsShapes = window._obsShapes || {{temporary:{{}}, permanent:{{}}}};
			window._obsLabels = window._obsLabels || {{temporary:{{}}, permanent:{{}}}};

			var cat = '{cat}';
			var oid = '{oid}';

			// Remove previous shape if it exists
			if (window._obsShapes[cat] && window._obsShapes[cat][oid]) {{
				try {{ m.removeLayer(window._obsShapes[cat][oid]); }} catch(e) {{}}
				window._obsShapes[cat][oid] = null;
			}}

			// Remove previous label if it exists
			if (window._obsLabels[cat] && window._obsLabels[cat][oid]) {{
				try {{ m.removeLayer(window._obsLabels[cat][oid]); }} catch(e) {{}}
				window._obsLabels[cat][oid] = null;
			}}

			var coords = {coords_json};
			if (coords.length < 2) return;

			var style = {{
				color: '{color}',
				weight: {weight},
				dashArray: {dash},
				fillOpacity: {fill_opacity}
			}};

			var layer;
			if (coords.length >= 3) {{
				layer = L.polygon(coords, style);
			}} else {{
				layer = L.polyline(coords, style);
			}}

			// LEFT-click selects / deselects this obstacle
			layer.on('click', function(e) {{
				if (window.pyBridge && window.pyBridge.selectObstacle) {{
					window.pyBridge.selectObstacle(oid, cat);
				}}
			}});

			// RIGHT-click deletes this obstacle
			layer.on('contextmenu', function(e) {{
				if (window.pyBridge && window.pyBridge.removeObstacle) {{
					window.pyBridge.removeObstacle(oid, cat);
				}}
			}});

			layer.addTo(m);
			window._obsShapes[cat][oid] = layer;

			// Center label with obstacle number
			var labelText = {json.dumps(label_text)};
			if (labelText) {{
				var center = [{lat_center}, {lng_center}];
				var labelIcon = L.divIcon({{
					className: 'obstacle-label',
					html: '<div style="background: white; border: 1px solid #333; border-radius: 12px; padding: 2px 6px; font-size: 11px; font-weight: bold; text-align: center;">' + labelText + '</div>',
					iconSize: [24, 24]
				}});
				var labelMarker = L.marker(center, {{icon: labelIcon, interactive: false}});
				labelMarker.addTo(m);
				window._obsLabels[cat][oid] = labelMarker;
			}}
		}})();
		"""
		self._get_page().runJavaScript(js)

	
	def _on_select_obstacle(self, oid: str, category: str):
		cat = 'temporary' if category.startswith('temporary') else 'permanent'
		# toggle selection
		if self.selected_obstacle[cat] == oid:
			self.selected_obstacle[cat] = None
		else:
			self.selected_obstacle[cat] = oid
		# redraw shapes of this category so weights/dashes update
		for k in list(self.obstacles[cat].keys()):
			self._redraw_obstacle_shape(cat, k)

	def _on_remove_obstacle(self, oid: str, category: str):
		cat = 'temporary' if category.startswith('temporary') else 'permanent'
		if oid not in self.obstacles[cat]:
			return

        # all points that belong to this obstacle
		pts = list(self.obstacles[cat].get(oid, []))

        # remove obstacle from in-memory structures
		self.obstacles[cat].pop(oid, None)
		if self.selected_obstacle[cat] == oid:
			self.selected_obstacle[cat] = None

        # Remove rows from the appropriate CSV (by obstacle_id)
		path = self.csv_perm if cat == 'permanent' else self.csv_temp
		try:
			with open(path, 'r', newline='') as src:
				r = csv.DictReader(src)
				fieldnames = r.fieldnames or ['id', 'timestamp', 'lat', 'lng', 'label', 'obstacle_id']
				kept = [row for row in r if row.get('obstacle_id') != oid]
			with open(path, 'w', newline='') as out:
				w = csv.DictWriter(out, fieldnames=fieldnames)
				w.writeheader()
				for row in kept:
					w.writerow(row)
		except Exception as e:
			print(f"[warn] removing obstacle from CSV failed: {e}")

        # Remove its points from in-memory indexes and from the map
		for p in pts:
			pid = p.get('pid')
			if not pid:
				continue
			self.point_to_obstacle.pop(pid, None)
			self.point_index.pop(pid, None)

            # remove the marker with this pid from the Leaflet group
			js_rm = f"""
            (function(){{
                var m = window['{self.map.jsName}Object'] || window.map;
                if (!m || !window._storedGroup) return;
                var toRemove = [];
                window._storedGroup.eachLayer(function(layer){{
                    if (layer && layer.options && layer.options.pid === '{pid}') {{
                        toRemove.push(layer);
                    }}
                }});
                toRemove.forEach(function(l){{ window._storedGroup.removeLayer(l); }});
            }})();
            """
			self._get_page().runJavaScript(js_rm)

        # clear this obstacle's shape and label from the map
		js = f"""
        (function(){{
            var m = window['{self.map.jsName}Object'] || window.map;
            if (!m) return;
            if (window._obsShapes && window._obsShapes['{cat}'] && window._obsShapes['{cat}']['{oid}']) {{
                try {{ m.removeLayer(window._obsShapes['{cat}']['{oid}']); }} catch(e){{}}
                window._obsShapes['{cat}']['{oid}'] = null;
            }}
            if (window._obsLabels && window._obsLabels['{cat}'] && window._obsLabels['{cat}']['{oid}']) {{
                try {{ m.removeLayer(window._obsLabels['{cat}']['{oid}']); }} catch(e){{}}
                window._obsLabels['{cat}']['{oid}'] = null;
            }}
        }})();
        """
		self._get_page().runJavaScript(js)

	def _update_obstacle_shape(self, label_key: str):
			"""
			Draw a line (2 pts) or a filled polygon (3+ pts) for the given label,
			using a CONVEX HULL to avoid self-intersections even if points are added
			out of order.
			"""
			label_key = (label_key or "").strip().lower()
			if label_key not in ("temporary_obstacle", "permanent_obstacle"):
				return

			verts = self.temp_vertices if label_key == "temporary_obstacle" else self.perm_vertices
			color = "#ff7f0e" if label_key == "temporary_obstacle" else "#d62728"  # orange/red
			layer_var = "_tempShapeLayer" if label_key == "temporary_obstacle" else "_permShapeLayer"

			if len(verts) < 2:
				# clear layer if it exists
				js = f"""
				(function() {{
				var m = window['{self.map.jsName}Object'] || window.map;
				if (!m) return;
				window.{layer_var} = window.{layer_var} || L.layerGroup().addTo(m);
				window.{layer_var}.clearLayers();
				}})();
				"""
				self._get_page().runJavaScript(js)
				return

			# ---- helper: monotone-chain convex hull in Python ----
			def _cross(o, a, b):
				return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

			pts = [(lat, lng) for (lat, lng) in verts]
			pts = sorted(set(pts))  # unique & sorted

			if len(pts) == 2:
				hull = pts  # just the segment
			else:
				lower = []
				for p in pts:
					while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
						lower.pop()
					lower.append(p)
				upper = []
				for p in reversed(pts):
					while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
						upper.pop()
					upper.append(p)
				hull = lower[:-1] + upper[:-1]  # counterclockwise hull

			coords = [[lat, lng] for (lat, lng) in hull]

			import json
			coords_json = json.dumps(coords)

			# ---- draw line (2) or polygon (3+) in Leaflet ----
			js = f"""
			(function() {{
			var m = window['{self.map.jsName}Object'] || window.map;
			if (!m) {{ console.error('Leaflet map not found'); return; }}
			window.{layer_var} = window.{layer_var} || L.layerGroup().addTo(m);
			var layer = window.{layer_var};
			layer.clearLayers();

			var coords = {coords_json};
			if (coords.length >= 3) {{
				L.polygon(coords, {{color: '{color}', weight: 2, fillOpacity: 0.25}}).addTo(layer);
			}} else {{
				L.polyline(coords, {{color: '{color}', weight: 2}}).addTo(layer);
			}}
			}})();
			"""
			self._get_page().runJavaScript(js)
	
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
		cat = None  # 'temporary' or 'permanent' for obstacles

		if key == 'permanent_obstacle':
			target = self.csv_perm; color = '#d62728'; tip_label = 'Permanent obstacle'; kind = 'perm'; cat = 'permanent'
		elif key == 'temporary_obstacle':
			target = self.csv_temp; color = '#ff7f0e'; tip_label = 'Temporary obstacle'; kind = 'temp'; cat = 'temporary'
		elif key == 'point_to_traverse':
			target = self.csv_trav; color = '#1f77b4'; tip_label = 'Point to traverse'; kind = 'trav'
		else:
			target = self.csv_temp; color = '#7f7f7f'; tip_label = f'Unknown: {label}'; kind = 'temp'

		# Unique point id
		pid = str(uuid.uuid4())
		obstacle_id = None

		# --- In-memory obstacle bookkeeping ---
		if cat is not None:
			# Add to currently selected obstacle, or start a new one
			obstacle_id = self.selected_obstacle.get(cat)
			if not obstacle_id:
				self.obstacle_counter[cat] += 1
				# obstacle_id is just the per-category number as a string: "1", "2", ...
				obstacle_id = str(self.obstacle_counter[cat])
				self.selected_obstacle[cat] = obstacle_id

			# Store this vertex under that obstacle
			self.obstacles[cat][obstacle_id].append({'pid': pid, 'lat': lat, 'lng': lng})
			self.point_to_obstacle[pid] = (cat, obstacle_id)

		# --- Persist to CSV ---
		row = {
			'id': pid,
			'timestamp': datetime.now().isoformat(timespec='seconds'),
			'lat': f'{lat:.8f}',
			'lng': f'{lng:.8f}',
			'label': key
		}
		if cat is not None and obstacle_id is not None:
			row['obstacle_id'] = obstacle_id
			fieldnames = ['id', 'timestamp', 'lat', 'lng', 'label', 'obstacle_id']
		else:
			fieldnames = ['id', 'timestamp', 'lat', 'lng', 'label']
		with open(target, 'a', newline='') as f:
			writer = csv.DictWriter(f, fieldnames=fieldnames)
			writer.writerow(row)

		self.point_index[pid] = (kind, str(target))

		# --- Draw marker; MIDDLE-CLICK to remove this point (unchanged logic) ---
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
				}}
				start();
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

		# --- Redraw just this obstacle's shape ---
		if cat is not None and obstacle_id is not None:
			self._redraw_obstacle_shape(cat, obstacle_id)

	def _on_remove_point(self, pid: str, lat: float, lng: float, label: str):
		"""
		Remove strictly by pid:
		1) copy the old CSV into a *_backups/ folder,
		2) overwrite the original CSV without the pid row,
		3) delete the *_backups/ folder after the write succeeds.
		"""
		# Prefer the known CSV path from our in-memory index; else scan all three.
		info = self.point_index.get(pid)
		candidate_paths = [info[1]] if info else [str(self.csv_perm), str(self.csv_temp), str(self.csv_trav)]

		for path in candidate_paths:
			if not os.path.exists(path):
				continue

			# Read and filter by pid
			try:
				with open(path, 'r', newline='') as src:
					r = csv.DictReader(src)
					fieldnames = r.fieldnames or ['id', 'timestamp', 'lat', 'lng', 'label']
					filtered_rows = []
					removed_here = False
					for row in r:
						if row.get('id') == pid:
							removed_here = True
							continue
						filtered_rows.append(row)
			except Exception as e:
				print(f"[error] Failed reading {path}: {e}")
				continue

			if not removed_here:
				# Nothing to remove in this file; try the next one
				continue

			# (1) Backup the old CSV before overwriting
			backups_dir = None
			try:
				backups_dir = Path(path).parent / (Path(path).stem + "_backups")
				backups_dir.mkdir(parents=True, exist_ok=True)
				backup_path = backups_dir / f"{Path(path).stem}_backup.csv"
				shutil.copy(path, backup_path)
			except Exception as e:
				print(f"[warn] Could not create backup for {path}: {e}")

			# (2) Overwrite with filtered content
			try:
				# Ensure 'id' remains in header
				if 'id' not in fieldnames:
					fieldnames = ['id', 'timestamp', 'lat', 'lng', 'label']
					for row in filtered_rows:
						row.setdefault('id', '')

				with open(path, 'w', newline='') as out:
					w = csv.DictWriter(out, fieldnames=fieldnames)
					w.writeheader()
					for row in filtered_rows:
						w.writerow(row)

				# (3) Delete the backup folder now that the write succeeded
				if backups_dir and backups_dir.exists():
					try:
						shutil.rmtree(backups_dir, ignore_errors=True)
					except Exception as e:
						print(f"[warn] Could not remove backup folder {backups_dir}: {e}")

			except Exception as e:
				print(f"[error] Failed writing {path}: {e}")
				# If write fails, we keep the backup folder.
				continue

			# If this was an obstacle point, update in-memory obstacle geometry
			label_key = (label or "").strip().lower()
			if label_key in ("temporary_obstacle", "permanent_obstacle"):
				cat = 'temporary' if label_key == "temporary_obstacle" else 'permanent'
				info = self.point_to_obstacle.pop(pid, None)
				if info:
					cat_from_map, oid = info
					cat = cat_from_map  # trust mapping
					pts = self.obstacles.get(cat, {}).get(oid, [])
					if pts:
						self.obstacles[cat][oid] = [p for p in pts if p.get('pid') != pid]
						self._redraw_obstacle_shape(cat, oid)

			# Success: clean up in-memory index and stop
			if pid in self.point_index:
				self.point_index.pop(pid, None)
			break

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

		# Button to load "places to go" points from CSV
		self.load_places_btn = QPushButton("Load Places To Go")
		self.load_places_btn.clicked.connect(self.load_places_to_go)

		# Button to load permanent and temporary obstacles from their CSVs
		self.load_obstacles_btn = QPushButton("Load Obstacles")
		self.load_obstacles_btn.clicked.connect(self._load_obstacles_from_csv_files)

		# Button to build straight-line path between "places to go"
		self.make_path_btn = QPushButton("Make Path")
		self.make_path_btn.clicked.connect(self.make_straight_line_path)

		layout = QVBoxLayout()
		layout.setContentsMargins(5, 0, 0, 0)

		# Buttons on top, map underneath
		layout.addWidget(self.load_places_btn)
		layout.addWidget(self.load_obstacles_btn)
		layout.addWidget(self.make_path_btn)
		layout.addWidget(self.mapWidget)

		self.setLayout(layout)

	def _add_obstacle_marker_from_csv(self, pid: str, lat: float, lng: float,
	                                  label_key: str, color: str, tip_label: str) -> None:
		"""
		Draw an obstacle vertex marker (same style as _on_store_point) without
		writing anything back to CSV.
		"""
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
				}}
				start();
			}}

			window._storedGroup = window._storedGroup || L.layerGroup().addTo(m);
			var mk = L.circleMarker([{lat}, {lng}], {{radius:6, color:'{color}', weight:2}});
			mk.bindTooltip('{tip_label} ({lat:.5f}, {lng:.5f})');
			mk.options.pid = '{pid}';

			// Middle-click removal (same as live points)
			mk.on('mousedown', function(e) {{
				var oe = e.originalEvent || e;
				if (oe && oe.button === 1) {{
					if (L && L.DomEvent) {{ L.DomEvent.preventDefault(oe); L.DomEvent.stop(oe); }}
					ensureBridge(function() {{
						if (window.pyBridge && window.pyBridge.removePoint) {{
							window.pyBridge.removePoint('{pid}', {lat}, {lng}, '{label_key}');
						}}
					}});
					m.removeLayer(mk);
				}}
			}});

			mk.addTo(window._storedGroup);
		}})();
		"""
		self._get_page().runJavaScript(js)

	def _load_obstacles_from_csv_files(self) -> None:
		"""
		Read the current session's permanent and temporary obstacle CSV files
		and load any obstacle points that are not already in memory/onscreen.

		After the points are placed, also draw the obstacle polygons/lines
		using those points.
		"""
		# category -> (csv_path, color, tip_label, label_key)
		configs = {
			'permanent': (
				self.csv_perm,
				'#d62728',          # same as in _on_store_point
				'Permanent obstacle',
				'permanent_obstacle',
			),
			'temporary': (
				self.csv_temp,
				'#ff7f0e',
				'Temporary obstacle',
				'temporary_obstacle',
			),
		}

		for cat, (csv_path, color, tip_label, label_key) in configs.items():
			if not csv_path or not Path(csv_path).exists():
				continue

			# keep track of which obstacle_ids we touched so we can redraw shapes
			touched_obstacles = set()

			try:
				with open(csv_path, newline='') as f:
					reader = csv.DictReader(f)
					for row in reader:
						if not row:
							continue

						pid = row.get('id') or str(uuid.uuid4())

						# If we've already loaded this point in this session, skip it
						if pid in self.point_index:
							continue

						try:
							lat = float(row.get('lat', ''))
							lng = float(row.get('lng', ''))
						except (TypeError, ValueError):
							continue

						oid = row.get('obstacle_id')
						if not oid:
							# If obstacle_id is missing, treat each point as its own obstacle
							oid = pid

						# --- in-memory bookkeeping (needed for shapes & delete) ---
						self.obstacles[cat][oid].append({'pid': pid, 'lat': lat, 'lng': lng})
						self.point_to_obstacle[pid] = (cat, oid)

						kind = 'perm' if cat == 'permanent' else 'temp'
						self.point_index[pid] = (kind, str(csv_path))

						# --- draw the vertex marker ---
						self._add_obstacle_marker_from_csv(pid, lat, lng, label_key, color, tip_label)

						touched_obstacles.add(oid)

				# Update the obstacle counter to at least the max existing id (if numeric)
				if self.obstacles[cat]:
					try:
						max_existing = max(
							int(k) for k in self.obstacles[cat].keys() if str(k).isdigit()
						)
						self.obstacle_counter[cat] = max(self.obstacle_counter[cat], max_existing)
					except ValueError:
						# some ids may be non-numeric; ignore for counter
						pass

				# --- now that points are placed, draw shapes for each obstacle we touched ---
				for oid in touched_obstacles:
					self._redraw_obstacle_shape(cat, oid)

			except Exception as e:
				print(f"[warn] Failed loading {cat} obstacles from {csv_path}: {e}")
	
	def load_places_to_go(self) -> None:
		"""
		Read places_to_go.csv (if it exists) and drop markers styled like
		obstacle vertices, but in a different color.
		On each press, remove previously loaded 'places to go' markers and
		recreate them from the file.
		THIS ONLY LOADS 'PLACES TO GO' POINTS; OBSTACLES ARE LOADED BY
		THE 'LOAD OBSTACLES' BUTTON.
		"""
		# Make sure our marker list exists
		if not hasattr(self, "places_to_go_markers"):
			self.places_to_go_markers = []

		# Python copy of the points (used for path planning)
		self.places_to_go_points = []

		# 1) Remove any previously created 'places to go' markers from the map
		for mk in self.places_to_go_markers:
			try:
				self.map.removeLayer(mk)
			except Exception as e:
				print(f"[warn] failed removing old places-to-go marker: {e}")
		self.places_to_go_markers.clear()

		# CSV expected in the same folder as map_viewer.py
		csv_path = Path(__file__).parent / "places_to_go.csv"

		if not csv_path.exists():
			QMessageBox.warning(
				self,
				"Places to go",
				f"CSV file with places to go was not found.\n\nExpected at:\n{csv_path}"
			)
			return

		added = 0

		try:
			with open(csv_path, newline='') as f:
				reader = csv.DictReader(f)

				for row in reader:
					if not row:
						continue

					# Name column (matches your sample: "Point Name")
					name = (
						row.get("Point Name")
						or row.get("name")
						or row.get("Name")
						or ""
					)

					# Latitude / longitude columns
					try:
						lat_str = row.get("Latitude") or row.get("lat") or row.get("Lat")
						lng_str = row.get("Longitude") or row.get("lon") or row.get("Lng") or row.get("Long")

						if lat_str is None or lng_str is None:
							continue

						lat = float(lat_str)
						lng = float(lng_str)
					except (TypeError, ValueError):
						# Skip any bad rows
						continue

					# save in python structure for path planning
					self.places_to_go_points.append({
						"name": name,
						"lat": lat,
						"lng": lng,
					})

					# --- Obstacle-style marker, but different color ---
					place_color = "#ffffff"  # distinct color for 'places to go'

					marker = L.circleMarker([lat, lng], {
						"radius": 6,
						"color": place_color,
						"weight": 2,
					})

					# Use the name as popup/tooltip, fall back to coordinates
					label = name or f"{lat:.6f}, {lng:.6f}"
					marker.bindPopup(label)

					marker.addTo(self.map)
					self.places_to_go_markers.append(marker)
					added += 1

			if added == 0:
				QMessageBox.information(
					self,
					"Places to go",
					f"No valid points were found in:\n{csv_path}"
				)
			else:
				QMessageBox.information(
					self,
					"Places to go",
					f"Loaded {added} point(s) from:\n{csv_path}"
				)

		except Exception as e:
			QMessageBox.critical(
				self,
				"Places to go",
				f"Failed to read CSV file:\n{csv_path}\n\nError: {e}"
			)

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

	'''def arm_radius(self, point: MapPoint):
		# grab data from the row that has Astronout (need to make sure that there is actually points stored on delivery_lat_long_goal)
		# go to that coordinate on the map, draw a circle with radius X 
		# Question: how does this function get called? 

		delivery_path = Path.cwd() + "/delivery_lat_lon_goal.csv"

		try:
			with open(delivery_path, newline="") as f:
				print("LOOKING FOR CSV")
				reader = csv.reader(f)
				for row in reader:
					if row[0] == "Astronaut 2":    
						target_lat = row[1]
						target_long = row[2]
				
			print("DRAWING CIRCLEEEEEE")
			c = L.circle([point.latitude, point.longitude], point.radius, {'weight': 4})

			if point.name:
				map_js_snnipets.map_layer_bind_tooltip(c, point.name, {'permanent': True, 'direction': 'down'})
			self.points_layer[layer].addLayer(c)
			self.point.addTo(self.map)

			self.draw_line_through_points(layer, points)
			

		except:
			print("No data is stored, please press Delivery button")'''

	def arm_radius(self):
		"""
		Reads the delivery_lat_lon_goal.csv file, finds the row for 'Astronaut 2',
		and draws a circle at that coordinate with the configured point radius.
		"""

		delivery_path = Path.cwd() / "delivery_lat_lon_goal.csv"
		print(delivery_path)

		try:
			target_lat = None
			target_long = None

			with open(delivery_path, newline="") as f:
				reader = csv.reader(f)

				for row in reader:
					# CSV format assumed: name, lat, lon
					if row[0] == "Astronaut 2":
						target_lat = float(row[1])
						target_long = float(row[2])
						break

			if target_lat is None:
				print("Astronaut 2 not found in CSV")
				print(target_lat)
				return

			print("DRAWING CIRCLE")
			
			# removing exisiting arm layer 
			if self.arm_layer:
				self.map.removeLayer(self.arm_layer)

			# creating new layer 
			self.arm_layer = L.layerGroup()
			self.arm_layer.addTo(self.map)


			# Draw the circle
			circle = L.circle([target_lat, target_long],{
					"radius":  50,  
					"fillColor": "red",
					"color":   "red",
					"weight":  4
				}
			)

			circle.addTo(self.arm_layer)

			'''# Optional tooltip
			if point.name:
				map_js_snnipets.map_layer_bind_tooltip(
					circle, 
					point.name,
					{"permanent": True, "direction": "down"}
				)'''

		except Exception as e:
			print("Error:", e)
			print("No data is stored — please press Delivery button")


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

	def hide_elevation(self):
		"""Remove elevation layer from the map."""
		if self.elevation_layer:
			self.map.removeLayer(self.elevation_layer)
			self.elevation_data: dict = {}
			self.elevation_layer = None
			self.elevation_min: float = 0
			self.elevation_max: float = 0

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
	#viewer.arm_radius()

	print("Calling arm_radius...")
	viewer.arm_radius()
	print("Returned from arm_radius.")


	
	# viewer.set_robot_position(38.5,-110.78)

	
	sys.exit(app.exec_())