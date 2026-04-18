#!/usr/bin/env python3
"""MissionState-based global path planner node.

Rover-side planner node:
- subscribes to mission_state for START_SL_AVOIDANCE requests,
- computes a path using PathPlanner,
- publishes GLOBAL_PATH_READY with global_planner_waypoints.
"""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from rover.msg import MissionState

import path_planner

LatLng = Tuple[float, float]
ObstacleStore = Dict[str, DefaultDict[str, List[Dict[str, float]]]]


class MissionStatePathPlanner(Node):
    OBSTACLE_CLEARANCE = 0.00005

    def __init__(self) -> None:
        super().__init__("mission_state_path_planner")

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._mission_pub = self.create_publisher(MissionState, "mission_state", qos)
        self._mission_sub = self.create_subscription(
            MissionState,
            "mission_state",
            self._mission_state_callback,
            qos,
        )

        self._planner = path_planner.PathPlanner(obstacle_clearance=self.OBSTACLE_CLEARANCE)
        self._search_dirs = [Path.cwd(), Path(__file__).resolve().parent]
        self._obstacles: ObstacleStore = {
            "temporary": defaultdict(list),
            "permanent": defaultdict(list),
        }

        self.get_logger().info("MissionState path planner is running.")

    def _mission_state_callback(self, msg: MissionState) -> None:
        if msg.state != "START_SL_AVOIDANCE":
            return

        start = (
            float(msg.current_pose.pose.position.x),
            float(msg.current_pose.pose.position.y),
        )
        goal = (
            float(msg.current_goal.pose.position.x),
            float(msg.current_goal.pose.position.y),
        )

        self._reload_obstacles_from_csv()
        hulls = self._planner.collect_hulls(self._obstacles)

        try:
            path = self._planner.plan_between_points(start, goal, hulls)
        except Exception as exc:
            self.get_logger().error(f"Planner exception: {exc}")
            path = [start, goal]

        if not path:
            path = [start, goal]

        deduped_path = self._dedupe_points(path)
        self._publish_ready(msg, deduped_path)

    def _publish_ready(self, request_msg: MissionState, points: List[LatLng]) -> None:
        out = MissionState()
        out.state = "GLOBAL_PATH_READY"
        out.current_state = request_msg.current_state
        out.current_goal = request_msg.current_goal
        out.current_pose = request_msg.current_pose
        out.global_planner_waypoints = self._flatten_points(points)
        self._mission_pub.publish(out)

        self.get_logger().info(
            f"Published GLOBAL_PATH_READY with {len(points)} point(s)."
        )

    @staticmethod
    def _flatten_points(points: List[LatLng]) -> List[float]:
        flattened: List[float] = []
        for lat, lng in points:
            flattened.extend([float(lat), float(lng)])
        return flattened

    @staticmethod
    def _dedupe_points(points: List[LatLng]) -> List[LatLng]:
        seen = set()
        out: List[LatLng] = []
        for lat, lng in points:
            key = (round(lat, 8), round(lng, 8))
            if key in seen:
                continue
            seen.add(key)
            out.append((lat, lng))
        return out

    def _reload_obstacles_from_csv(self) -> None:
        self._obstacles = {
            "temporary": defaultdict(list),
            "permanent": defaultdict(list),
        }

        self._load_obstacle_category("permanent", self._find_latest_csv("permanent_obstacles_*.csv"))
        self._load_obstacle_category("temporary", self._find_latest_csv("temporary_obstacles_*.csv"))

    def _find_latest_csv(self, pattern: str) -> Optional[Path]:
        matches: List[Path] = []
        for directory in self._search_dirs:
            try:
                matches.extend(directory.glob(pattern))
            except Exception:
                continue

        if not matches:
            return None

        matches.sort(key=lambda p: p.stat().st_mtime)
        return matches[-1]

    def _load_obstacle_category(self, category: str, csv_path: Optional[Path]) -> None:
        if csv_path is None or not csv_path.exists():
            return

        try:
            with open(csv_path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if not row:
                        continue

                    try:
                        lat = float(row.get("lat", ""))
                        lng = float(row.get("lng", ""))
                    except (TypeError, ValueError):
                        continue

                    oid = row.get("obstacle_id") or row.get("id") or "unknown"
                    self._obstacles[category][str(oid)].append({"lat": lat, "lng": lng})
        except Exception as exc:
            self.get_logger().warn(f"Failed to load {category} obstacles from {csv_path}: {exc}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionStatePathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
