#!/usr/bin/env python3
"""Display-focused map viewer for the base station GUI.

This module intentionally keeps planning out of the GUI widget. It listens for
planner outputs on MissionState and renders the returned global path.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QMessageBox
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from rover.msg import MissionState

import map_js_snnipets
import map_viewer as base_map_viewer

LatLng = Tuple[float, float]


class MapViewerDisplay(base_map_viewer.MapViewer):
    """Base-station map widget that only displays planner output.

    Path requests and path responses are exchanged through MissionState.
    """

    def __init__(self, ros_node: Optional[Node] = None, *args, **kwargs) -> None:
        self._ros_node = ros_node
        self._mission_pub = None
        self._mission_sub = None

        self.current_path_line = None
        self.current_path_points: List[List[float]] = []
        self.old_path_lines = []

        super().__init__(*args, **kwargs)

        self._configure_path_button()
        self._setup_mission_state_io()

    def _configure_path_button(self) -> None:
        if not hasattr(self, "make_path_btn"):
            return

        try:
            self.make_path_btn.clicked.disconnect()
        except TypeError:
            pass

        self.make_path_btn.setText("Request Path")
        self.make_path_btn.clicked.connect(self.request_path_for_loaded_points)

    def _setup_mission_state_io(self) -> None:
        if self._ros_node is None:
            return

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._mission_pub = self._ros_node.create_publisher(MissionState, "mission_state", qos)
        self._mission_sub = self._ros_node.create_subscription(
            MissionState,
            "mission_state",
            self._mission_state_callback,
            qos,
        )

    def _mission_state_callback(self, msg: MissionState) -> None:
        if msg.state == "START_SL_AVOIDANCE":
            self.clear_global_path()
            return

        if msg.state != "GLOBAL_PATH_READY":
            return

        waypoints = self._deserialize_waypoints(msg.global_planner_waypoints)
        if not waypoints:
            return
        self.draw_global_path(waypoints)

    @staticmethod
    def _deserialize_waypoints(flat: List[float]) -> List[LatLng]:
        points: List[LatLng] = []
        if not flat:
            return points

        usable_len = len(flat) - (len(flat) % 2)
        for i in range(0, usable_len, 2):
            points.append((float(flat[i]), float(flat[i + 1])))
        return points

    @staticmethod
    def _serialize_waypoints(points: List[LatLng]) -> List[float]:
        flat: List[float] = []
        for lat, lng in points:
            flat.extend([float(lat), float(lng)])
        return flat

    def _ensure_current_path_line(self) -> None:
        if self.current_path_line is not None:
            return

        self.current_path_line = base_map_viewer.L.polyline([], {"color": "#00ff00"})
        self.current_path_line.addTo(self.map)

    def draw_global_path(self, path_points: List[LatLng]) -> None:
        latlngs = [[lat, lng] for lat, lng in path_points]

        if self.current_path_points:
            history_line = base_map_viewer.L.polyline(self.current_path_points, {"color": "#ff8800"})
            history_line.addTo(self.map)
            self.old_path_lines.append(history_line)

        self._ensure_current_path_line()
        map_js_snnipets.map_marker_set_lat_lng_points(self.map, [], self.current_path_line.jsName)
        map_js_snnipets.map_marker_set_lat_lng_points(self.map, latlngs, self.current_path_line.jsName)
        self.current_path_points = latlngs

    def clear_global_path(self) -> None:
        if self.current_path_line is not None:
            map_js_snnipets.map_marker_set_lat_lng_points(self.map, [], self.current_path_line.jsName)
        self.current_path_points = []

        if self.old_path_lines:
            for line in self.old_path_lines:
                try:
                    self.map.removeLayer(line)
                except Exception:
                    pass
            self.old_path_lines = []

    def request_path_for_loaded_points(self) -> None:
        if self._mission_pub is None:
            QMessageBox.information(self, "Path Planner", "No ROS node attached for MissionState publishing.")
            return

        pts = getattr(self, "places_to_go_points", None)
        if not pts or len(pts) < 2:
            QMessageBox.information(
                self,
                "Path Planner",
                "Load at least two points before requesting a path.",
            )
            return

        start_idx = 0
        for i, p in enumerate(pts):
            if (p.get("name") or "").strip().lower() == "start":
                start_idx = i
                break

        start = (float(pts[start_idx]["lat"]), float(pts[start_idx]["lng"]))
        goal = (float(pts[-1]["lat"]), float(pts[-1]["lng"]))

        msg = MissionState()
        msg.state = "START_SL_AVOIDANCE"
        msg.current_state = "GUI_REQUEST"

        msg.current_pose = PoseStamped()
        msg.current_pose.pose.position.x = start[0]
        msg.current_pose.pose.position.y = start[1]

        msg.current_goal = PoseStamped()
        msg.current_goal.pose.position.x = goal[0]
        msg.current_goal.pose.position.y = goal[1]

        msg.targets = self._serialize_waypoints([start, goal])
        self._mission_pub.publish(msg)


class MapViewer(MapViewerDisplay):
    """Compatibility alias expected by gui_pyqt."""

    pass
