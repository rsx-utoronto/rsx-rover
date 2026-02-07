#!/usr/bin/env python3
import sys
import os
import glob
import csv
import json
import time
from datetime import datetime
from typing import Dict, Any, List, Optional, Tuple

from rclpy.executors import SingleThreadedExecutor
from rsx_rover_sim.planning.global_planner_node import GlobalPlannerNode

from rsx_rover_sim.sim.rover_sim_node import RoverSimNode

from rsx_rover_sim.perception.fake_zed_detector_node import FakeZedDetectorNode
from rsx_rover_sim.planning.local_detour_planner_node import LocalDetourPlannerNode

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String

from PyQt5.QtCore import QUrl, QTimer, QObject, pyqtSlot, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QDockWidget, QListWidget, QListWidgetItem,
    QPushButton, QLabel, QWidget, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel

from rsx_rover_sim.planning.global_planner_node import GlobalPlannerNode
from rsx_rover_sim.sim.rover_sim_node import RoverSimNode



SW_LAT, SW_LNG = 38.409961, -110.790467
NE_LAT, NE_LNG = 38.411895, -110.786068


HTML_TEMPLATE = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, height=device-height, initial-scale=1.0" />
  <title>RSX Rover Simulation Map (Leaflet + ROS2)</title>

  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

  <!-- Leaflet.draw (polygon draw/edit/delete) -->
  <link rel="stylesheet" href="https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.css"/>
  <script src="https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.js"></script>

  <!-- Qt WebChannel (lets JS call Python slots) -->
  <script src="qrc:///qtwebchannel/qwebchannel.js"></script>

  <style>
    html, body {
      height: 100%;
      width: 100%;
      margin: 0;
      overflow: hidden;
    }
    #map {
      height: 100%;
      width: 100%;
      background: #f2f2f2;
    }
    #hoverBox {
      position: absolute;
      left: 10px;
      bottom: 10px;
      z-index: 9999;
      background: rgba(255, 255, 255, 0.85);
      padding: 6px 8px;
      border-radius: 6px;
      border: 1px solid rgba(0,0,0,0.2);
      font-family: sans-serif;
      font-size: 12px;
      user-select: none;
      pointer-events: none;
    }
    .tool-btn {
      display: block;
      width: 170px;
      padding: 0 12px;
      font: 13px/30px sans-serif;
      text-align: center;
      color: #111;
      text-decoration: none;
      background: rgba(255,255,255,0.95);
    }
    .tool-btn:hover {
      background: rgba(230,230,230,0.95);
    }
  </style>
</head>

<body>
  <div id="map"></div>
  <div id="hoverBox">Hover: —</div>

  <script>
    const sw = L.latLng(__SW_LAT__, __SW_LNG__);
    const ne = L.latLng(__NE_LAT__, __NE_LNG__);
    const bounds = L.latLngBounds(sw, ne);

    const map = L.map('map', {
      zoomControl: false,
      attributionControl: false,
      scrollWheelZoom: true,
      doubleClickZoom: false,
      touchZoom: false,
      boxZoom: false,
      keyboard: false,
      inertia: false
    });

    map.fitBounds(bounds, { padding: [60, 60] });
    map.setMaxBounds(bounds);
    map.options.maxBoundsViscosity = 1.0;

    const fittedZoom = map.getZoom();
    map.setMinZoom(fittedZoom);

    const hoverBox = document.getElementById('hoverBox');
    function fmt(n) { return Number(n).toFixed(6); }

    map.on('mousemove', function(e) {
      hoverBox.textContent = 'Hover: lat ' + fmt(e.latlng.lat) + ', lon ' + fmt(e.latlng.lng);
    });
    map.on('mouseout', function() { hoverBox.textContent = 'Hover: —'; });

    // Layers
    const placesLayer = L.layerGroup().addTo(map);

    const permObsLayer = new L.FeatureGroup();
    map.addLayer(permObsLayer);

    const permObsVertexLayer = L.layerGroup().addTo(map);

    // Temporary obstacles (Step 7)
    const tempObsLayer = new L.FeatureGroup();
    map.addLayer(tempObsLayer);

    const tempObsPolys = {};     // id -> polygon
    let nextTempObsId = 1;
    let drawMode = 'permanent';  // 'permanent' | 'temporary'

    // Temp obstacle vertices + selection
    const tempObsVertexLayer = L.layerGroup().addTo(map);
    const tempVerts = {};            // id -> [L.marker, ...] in ORDER
    let selectedTempId = null;

    const pathLayer = L.layerGroup().addTo(map);
    let globalPathLine = null;
    let activePathLine = null;

    // Detected visible temp obstacle wall segments (green)
    const detectedWallLayer = L.layerGroup().addTo(map);
    let detectedWallLines = [];

    // Fake ZED detections overlay
    const perceptionLayer = L.layerGroup().addTo(map);
    let perceptionDots = [];
    let perceptionNearest = null;

    window.updatePerception = function(payload) {
      // clear old
      perceptionDots.forEach(m => perceptionLayer.removeLayer(m));
      perceptionDots = [];
      if (perceptionNearest) { perceptionLayer.removeLayer(perceptionNearest); perceptionNearest = null; }

      if (!payload || !payload.boundary_points) return;

      // boundary points are in rover frame (x forward, y left).
      // Convert rover-frame meters -> lat/lon by using rover pose + yaw (in JS we only have pose lat/lon/yaw).
      // Easiest for now: just show count in console unless you want full transform.
      // (If you want full transform, tell me and I’ll add it cleanly.)
      console.log("Perception blocked:", payload.blocked, "points:", payload.boundary_points.length);

      // Optional: if payload.nearest exists, log it
      if (payload.nearest) console.log("Nearest:", payload.nearest);
    };

    // Rover display (Step 6)
    const roverLayer = L.layerGroup().addTo(map);
    let roverMarker = null;
    let roverTrail = null;
    let trailPts = [];

    // Rover vision cone (Step 7.5)
    let roverCone = null;
    let roverHeadingLine = null;

    const VISION_DEG = 150.0;
    const VISION_HALF_RAD = (VISION_DEG * Math.PI / 180.0) / 2.0; // 75°
    const VISION_RANGE_M = 25.0;   // <-- change this to make the cone longer/shorter
    const VISION_ARC_STEPS = 18;   // smoothness of the arc

    let lastYaw = 0.0;

    // Mission markers
    let startMarker = null;
    let goalMarker = null;

    // obstacle id -> [markers]
    const obsVerts = {};
    // obstacle id -> polygon
    const obsPolys = {};

    let nextObstacleId = 1;
    let selectedObsId = null;

    function makeVertexIcon() {
      return L.divIcon({
        className: '',
        html: '<div style="width:10px;height:10px;border:2px solid #3388ff;background:white;border-radius:50%;"></div>',
        iconSize: [10, 10],
        iconAnchor: [5, 5]
      });
    }

    function makeTempVertexIcon() {
      return L.divIcon({
        className: '',
        html: '<div style="width:10px;height:10px;border:2px solid #d00000;background:white;border-radius:50%;"></div>',
        iconSize: [10, 10],
        iconAnchor: [5, 5]
      });
    }

    function applyTempSelectionStyles() {
      // Polygons: highlight selected temp
      Object.keys(tempObsPolys).forEach(function(id) {
        const poly = tempObsPolys[id];
        const isSel = (selectedTempId !== null && id === selectedTempId);
        poly.setStyle({
          color: isSel ? '#ff7a00' : '#d00000',
          weight: isSel ? 5 : 3,
          fillOpacity: isSel ? 0.30 : 0.25
        });
      });

      // Vertex markers: only selected temp obstacle is draggable + bright
      Object.keys(tempVerts).forEach(function(id) {
        const markers = tempVerts[id] || [];
        const isSel = (selectedTempId !== null && id === selectedTempId);
        markers.forEach(function(m) {
          const op = (selectedTempId === null) ? 0.6 : (isSel ? 1.0 : 0.12);
          m.setOpacity(op);
          if (m.dragging) {
            if (isSel) m.dragging.enable();
            else m.dragging.disable();
          }
        });
      });
    }

    function setSelectedTempObstacle(tempId) {
      selectedTempId = (tempId === null || tempId === undefined || tempId === '') ? null : String(tempId);
      if (selectedTempId) {
        // selecting temp deselects permanent
        selectedObsId = null;
        applySelectionStyles();
      }
      applyTempSelectionStyles();
    }

    function applySelectionStyles() {
      Object.keys(obsPolys).forEach(function(id) {
        const poly = obsPolys[id];
        const isSel = (selectedObsId !== null && id === selectedObsId);
        poly.setStyle({
          color: isSel ? '#ff7a00' : '#3388ff',
          weight: isSel ? 5 : 3,
          fillOpacity: isSel ? 0.30 : 0.20
        });
      });

      Object.keys(obsVerts).forEach(function(id) {
        const markers = obsVerts[id] || [];
        const isSel = (selectedObsId !== null && id === selectedObsId);
        markers.forEach(function(m) {
          const op = (selectedObsId === null) ? 0.6 : (isSel ? 1.0 : 0.12);
          m.setOpacity(op);
          if (m.dragging) {
            if (isSel) m.dragging.enable();
            else m.dragging.disable();
          }
        });
      });
    }

    function setSelectedObstacle(obsId) {
      selectedObsId = (obsId === null || obsId === undefined || obsId === '') ? null : String(obsId);
      if (selectedObsId) {
        // selecting permanent deselects temp
        selectedTempId = null;
        applyTempSelectionStyles();
      }
      applySelectionStyles();
    }

    map.on('click', function(e) {
      // Shift+click adds a vertex to selected TEMP obstacle
      if (selectedTempId && e.originalEvent && e.originalEvent.shiftKey) {
        insertTempVertexAtClosestEdge(selectedTempId, e.latlng);
        rebuildTempObstaclePolygon(selectedTempId);
        applyTempSelectionStyles();
        publishTemporaryObstacles();
        return;
      }

      // Shift+click adds a vertex to selected PERMANENT obstacle
      if (selectedObsId && e.originalEvent && e.originalEvent.shiftKey) {
        addVertexMarker(selectedObsId, e.latlng.lat, e.latlng.lng);
        rebuildObstaclePolygon(selectedObsId);
        applySelectionStyles();
        return;
      }

      // click-away deselects both
      setSelectedObstacle(null);
      setSelectedTempObstacle(null);
    });

    // --- geometry helpers (JS) ---
    function dedupeLatLng(latlngs) {
      const seen = new Set();
      const out = [];
      for (const ll of latlngs) {
        const key = ll[0].toFixed(7) + "," + ll[1].toFixed(7);
        if (!seen.has(key)) {
          seen.add(key);
          out.push(ll);
        }
      }
      return out;
    }

    function convexHull(points) {
      if (points.length <= 1) return points.slice();
      const pts = points.slice().sort((a, b) => (a.x - b.x) || (a.y - b.y));
      const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);

      const lower = [];
      for (const p of pts) {
        while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
        lower.push(p);
      }
      const upper = [];
      for (let i = pts.length - 1; i >= 0; i--) {
        const p = pts[i];
        while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
        upper.push(p);
      }
      upper.pop(); lower.pop();
      return lower.concat(upper);
    }

    function styleObstacleLayer(layer) {
      layer.setStyle && layer.setStyle({ color: '#3388ff', weight: 3, fillOpacity: 0.2 });
    }

    function labelFor(layer) {
      const oid = layer._obsId ?? '?';
      return 'permanent #' + oid;
    }

    function getObstaclePoints(obsId) {
      const markers = obsVerts[obsId] || [];
      return markers.map(m => {
        const ll = m.getLatLng();
        return [ll.lat, ll.lng];
      });
    }

    function rebuildObstaclePolygon(obsId) {
      const latlngs = getObstaclePoints(obsId);
      if (latlngs.length < 3) {
        if (obsPolys[obsId]) {
          permObsLayer.removeLayer(obsPolys[obsId]);
          delete obsPolys[obsId];
        }
        applySelectionStyles();
        return;
      }

      const pts = latlngs.map(ll => ({ x: ll[1], y: ll[0] }));
      const hull = convexHull(pts);
      const hullLatLngs = hull.map(p => [p.y, p.x]);

      if (!obsPolys[obsId]) {
        const poly = L.polygon(hullLatLngs);
        poly._obsId = String(obsId);
        styleObstacleLayer(poly);
        poly.bindTooltip(labelFor(poly), { permanent: false });
        poly.on('click', function(ev) {
          setSelectedObstacle(obsId);
          if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
        });
        permObsLayer.addLayer(poly);
        obsPolys[obsId] = poly;
      } else {
        obsPolys[obsId].setLatLngs(hullLatLngs);
        styleObstacleLayer(obsPolys[obsId]);
      }
      applySelectionStyles();
    }

    function addVertexMarker(obsId, lat, lon) {
      obsId = String(obsId);
      obsVerts[obsId] = obsVerts[obsId] || [];

      const m = L.marker([lat, lon], {
        icon: makeVertexIcon(),
        draggable: true,
        keyboard: false
      });

      m.on('click', function(ev) {
        setSelectedObstacle(obsId);
        if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
      });

      // Only selected obstacle is draggable
      if (m.dragging) m.dragging.disable();

      m.on('drag', function() {
        rebuildObstaclePolygon(obsId);
      });

      m.on('contextmenu', function(ev) {
        if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
        permObsVertexLayer.removeLayer(m);
        obsVerts[obsId] = (obsVerts[obsId] || []).filter(x => x !== m);
        rebuildObstaclePolygon(obsId);
        applySelectionStyles();
      });

      permObsVertexLayer.addLayer(m);
      obsVerts[obsId].push(m);
    }

    function buildPermanentPayload() {
      const obstacles = [];
      Object.keys(obsVerts).forEach(function(obsId) {
        const markers = obsVerts[obsId] || [];
        if (markers.length === 0) return;
        const pts = markers.map(m => {
          const ll = m.getLatLng();
          return { lat: ll.lat, lon: ll.lng };
        });
        obstacles.push({ id: String(obsId), label: 'permanent', points: pts });
      });
      obstacles.sort((a, b) => (parseInt(a.id, 10) || 0) - (parseInt(b.id, 10) || 0));
      return { obstacles: obstacles };
    }

    function styleTempObstacleLayer(layer) {
      layer.setStyle && layer.setStyle({ color: '#d00000', weight: 3, fillOpacity: 0.25 });
    }

    function removeTempObstacle(tempId) {
      tempId = String(tempId);

      if (tempObsPolys[tempId]) {
        tempObsLayer.removeLayer(tempObsPolys[tempId]);
        delete tempObsPolys[tempId];
      }

      if (tempVerts[tempId]) {
        tempVerts[tempId].forEach(m => tempObsVertexLayer.removeLayer(m));
        delete tempVerts[tempId];
      }

      if (selectedTempId === tempId) selectedTempId = null;

      applyTempSelectionStyles();
      publishTemporaryObstacles();
    }

    function rebuildTempObstaclePolygon(tempId) {
      tempId = String(tempId);
      const markers = tempVerts[tempId] || [];
      const latlngs = markers.map(m => {
        const ll = m.getLatLng();
        return [ll.lat, ll.lng];
      });

      if (latlngs.length < 3) {
        if (tempObsPolys[tempId]) {
          tempObsLayer.removeLayer(tempObsPolys[tempId]);
          delete tempObsPolys[tempId];
        }
        applyTempSelectionStyles();
        return;
      }

      if (!tempObsPolys[tempId]) {
        const poly = L.polygon(latlngs);
        poly._tempId = tempId;
        styleTempObstacleLayer(poly);

        poly.on('click', function(ev) {
          setSelectedTempObstacle(tempId);
          if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
        });

        // right-click polygon deletes whole temp obstacle
        poly.on('contextmenu', function(ev) {
          if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
          removeTempObstacle(tempId);
        });

        tempObsLayer.addLayer(poly);
        tempObsPolys[tempId] = poly;
      } else {
        tempObsPolys[tempId].setLatLngs(latlngs);
        styleTempObstacleLayer(tempObsPolys[tempId]);
      }

      applyTempSelectionStyles();
    }

    function addTempVertexMarker(tempId, lat, lon, insertAt = null) {
      tempId = String(tempId);
      tempVerts[tempId] = tempVerts[tempId] || [];

      const m = L.marker([lat, lon], {
        icon: makeTempVertexIcon(),
        draggable: true,
        keyboard: false
      });

      m.on('click', function(ev) {
        setSelectedTempObstacle(tempId);
        if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
      });

      // only selected temp obstacle is draggable
      if (m.dragging) m.dragging.disable();

      m.on('drag', function() {
        rebuildTempObstaclePolygon(tempId);
      });

      // publish after dragging finishes (less spam)
      m.on('dragend', function() {
        publishTemporaryObstacles();
      });

      // right-click vertex deletes that vertex
      m.on('contextmenu', function(ev) {
        if (ev && ev.originalEvent) L.DomEvent.stopPropagation(ev.originalEvent);
        tempObsVertexLayer.removeLayer(m);
        tempVerts[tempId] = (tempVerts[tempId] || []).filter(x => x !== m);
        rebuildTempObstaclePolygon(tempId);
        publishTemporaryObstacles();
      });

      tempObsVertexLayer.addLayer(m);

      if (insertAt === null || insertAt === undefined || insertAt < 0 || insertAt > tempVerts[tempId].length) {
        tempVerts[tempId].push(m);
      } else {
        tempVerts[tempId].splice(insertAt, 0, m);
      }

      return m;
    }

    function _distPointToSegment(latlng, a, b) {
      // distance in "lat/lon space" (good enough for picking closest edge in your tiny map)
      const px = latlng.lng, py = latlng.lat;
      const ax = a.lng, ay = a.lat;
      const bx = b.lng, by = b.lat;

      const vx = bx - ax, vy = by - ay;
      const wx = px - ax, wy = py - ay;

      const vv = vx*vx + vy*vy;
      if (vv < 1e-18) {
        const dx = px - ax, dy = py - ay;
        return Math.sqrt(dx*dx + dy*dy);
      }

      let t = (wx*vx + wy*vy) / vv;
      t = Math.max(0, Math.min(1, t));
      const cx = ax + t*vx, cy = ay + t*vy;
      const dx = px - cx, dy = py - cy;
      return Math.sqrt(dx*dx + dy*dy);
    }

    function insertTempVertexAtClosestEdge(tempId, latlng) {
      tempId = String(tempId);
      const markers = tempVerts[tempId] || [];
      if (markers.length < 2) {
        addTempVertexMarker(tempId, latlng.lat, latlng.lng);
        return;
      }

      // Find best edge to insert into (treat polygon as closed)
      let bestIdx = markers.length; // default append
      let bestD = Infinity;

      for (let i = 0; i < markers.length; i++) {
        const a = markers[i].getLatLng();
        const b = markers[(i + 1) % markers.length].getLatLng();
        const d = _distPointToSegment(latlng, a, b);
        if (d < bestD) {
          bestD = d;
          bestIdx = i + 1;
        }
      }

      addTempVertexMarker(tempId, latlng.lat, latlng.lng, bestIdx);
    }

    function buildTemporaryPayload() {
      const obstacles = [];

      Object.keys(tempVerts).forEach(function(id) {
        const markers = tempVerts[id] || [];
        if (markers.length < 3) return;

        const pts = markers.map(m => {
          const ll = m.getLatLng();
          return { lat: ll.lat, lon: ll.lng };
        });

        obstacles.push({ id: String(id), label: 'temporary', points: pts });
      });

      obstacles.sort((a, b) => (parseInt(a.id, 10) || 0) - (parseInt(b.id, 10) || 0));
      return { obstacles: obstacles };
    }

    function publishTemporaryObstacles() {
      if (rosBridge && rosBridge.publishTemporaryObstacles) {
        const payload = buildTemporaryPayload();
        rosBridge.publishTemporaryObstacles(JSON.stringify(payload));
      }
    }

    function clearTemporaryObstacles() {
      tempObsLayer.clearLayers();
      tempObsVertexLayer.clearLayers();

      for (const k in tempObsPolys) delete tempObsPolys[k];
      for (const k in tempVerts) delete tempVerts[k];

      setSelectedTempObstacle(null);
      publishTemporaryObstacles();
    }

    // ---------- Python -> JS updates ----------
    window.updatePlaces = function(payload) {
      placesLayer.clearLayers();
      if (!payload || !payload.places) return;
      payload.places.forEach(function(p) {
        const m = L.circleMarker([p.lat, p.lon], { radius: 6 });
        m.addTo(placesLayer);
        const name = p.name ? p.name : 'Place';
        m.bindTooltip(name, { permanent: false, direction: 'top' });
      });
    };

    function offsetLatLon(lat, lon, dx_m, dy_m) {
      // dx_m: meters east, dy_m: meters north
      const R = 6371000.0;
      const dLat = (dy_m / R) * (180.0 / Math.PI);
      const dLon = (dx_m / (R * Math.cos(lat * Math.PI / 180.0))) * (180.0 / Math.PI);
      return [lat + dLat, lon + dLon];
    }

    function buildVisionCone(lat, lon, yawRad) {
      const pts = [];
      pts.push([lat, lon]); // apex at rover

      const start = yawRad - VISION_HALF_RAD;
      const end = yawRad + VISION_HALF_RAD;

      for (let i = 0; i <= VISION_ARC_STEPS; i++) {
        const t = i / VISION_ARC_STEPS;
        const ang = start + (end - start) * t;

        const dx = VISION_RANGE_M * Math.cos(ang);
        const dy = VISION_RANGE_M * Math.sin(ang);

        pts.push(offsetLatLon(lat, lon, dx, dy));
      }
      return pts;
    }

    window.updateRoverPose = function(payload) {
      if (!payload || payload.lat === undefined || payload.lon === undefined) return;

      const latlng = [payload.lat, payload.lon];

      // yaw is radians, 0=east, +CCW (matches rover_sim_node)
      if (payload.yaw !== undefined && payload.yaw !== null) {
        lastYaw = Number(payload.yaw);
      }

      const coneLatLngs = buildVisionCone(payload.lat, payload.lon, lastYaw);
      const headEnd = offsetLatLon(
        payload.lat, payload.lon,
        VISION_RANGE_M * Math.cos(lastYaw),
        VISION_RANGE_M * Math.sin(lastYaw)
      );

      // cone polygon
      if (!roverCone) {
        roverCone = L.polygon(coneLatLngs, {
          color: '#00a000',
          weight: 2,
          opacity: 0.6,
          fillOpacity: 0.10,
          interactive: false
        }).addTo(roverLayer);
      } else {
        roverCone.setLatLngs(coneLatLngs);
      }

      // center heading line
      if (!roverHeadingLine) {
        roverHeadingLine = L.polyline([[payload.lat, payload.lon], headEnd], {
          weight: 2,
          opacity: 0.8,
          dashArray: '6 6',
          interactive: false
        }).addTo(roverLayer);
      } else {
        roverHeadingLine.setLatLngs([[payload.lat, payload.lon], headEnd]);
      }

      // rover marker
      if (!roverMarker) {
        roverMarker = L.circleMarker(latlng, { radius: 8 }).addTo(roverLayer);
        roverMarker.bindTooltip('ROVER', { permanent: false });
      } else {
        roverMarker.setLatLng(latlng);
      }

      // trail (breadcrumb path)
      trailPts.push(latlng);
      if (trailPts.length > 2000) trailPts.shift(); // keep it bounded

      if (!roverTrail) {
        roverTrail = L.polyline(trailPts, { weight: 2, opacity: 0.7 }).addTo(roverLayer);
      } else {
        roverTrail.setLatLngs(trailPts);
      }

      if (roverMarker && roverMarker.bringToFront) roverMarker.bringToFront();
    };

    window.updatePermanentObstacles = function(payload) {
      permObsLayer.clearLayers();
      permObsVertexLayer.clearLayers();
      for (const k in obsVerts) delete obsVerts[k];
      for (const k in obsPolys) delete obsPolys[k];

      if (!payload || !payload.obstacles) return;

      let maxId = 0;
      payload.obstacles.forEach(function(o) {
        const oid = (o.id !== undefined && o.id !== null) ? String(o.id) : '';
        const asInt = parseInt(oid, 10);
        if (!isNaN(asInt)) maxId = Math.max(maxId, asInt);

        if (!o.points || o.points.length === 0) return;

        const useId = oid || String(maxId + 1);
        o.points.forEach(pt => addVertexMarker(useId, pt.lat, pt.lon));
        rebuildObstaclePolygon(useId);
      });

      nextObstacleId = Math.max(nextObstacleId, maxId + 1);
      setSelectedObstacle(null);
    };

    // Draw the GLOBAL path as a thin dashed line (debug overlay)
    window.updateGlobalPath = function(payload) {
      if (globalPathLine) {
        pathLayer.removeLayer(globalPathLine);
        globalPathLine = null;
      }

      if (!payload || !payload.path || payload.path.length < 2) return;

      const latlngs = payload.path.map(p => [p.lat, p.lon]);
      globalPathLine = L.polyline(latlngs, {
        color: '#444',
        weight: 2,
        opacity: 0.6,
        dashArray: '6 10'
      }).addTo(pathLayer);

      if (globalPathLine.bringToFront) globalPathLine.bringToFront();
    };

    window.updatePath = function(payload) {
      if (activePathLine) {
        pathLayer.removeLayer(activePathLine);
        activePathLine = null;
      }

      // Clear rover breadcrumb trail when a new path is drawn
      trailPts = [];
      if (roverTrail) {
        roverLayer.removeLayer(roverTrail);
        roverTrail = null;
      }
    
    window.updateDetectedWalls = function(payload) {
      // Clear old
      detectedWallLines.forEach(l => detectedWallLayer.removeLayer(l));
      detectedWallLines = [];

      if (!payload || !payload.walls_latlon) return;

      const walls = payload.walls_latlon;
      if (!Array.isArray(walls)) return;

      walls.forEach((wall) => {
        if (!Array.isArray(wall) || wall.length < 2) return;

        const latlngs = wall.map(p => [p.lat, p.lon]);

        const line = L.polyline(latlngs, {
          color: '#00cc00',
          weight: 5,
          opacity: 0.95
        }).addTo(detectedWallLayer);

        detectedWallLines.push(line);
      });

      // Keep these on top so you can clearly see what is detected
      detectedWallLines.forEach(l => l.bringToFront && l.bringToFront());
      if (activePathLine && activePathLine.bringToFront) activePathLine.bringToFront();
      if (globalPathLine && globalPathLine.bringToFront) globalPathLine.bringToFront();
    };

      if (!payload || !payload.path || payload.path.length < 2) return;

      const latlngs = payload.path.map(p => [p.lat, p.lon]);
      activePathLine = L.polyline(latlngs, {
        color: '#0066ff',
        weight: 4,
        opacity: 0.9
      }).addTo(pathLayer);

      // Keep layering consistent: global dashed behind, active on top
      if (activePathLine && activePathLine.bringToFront) activePathLine.bringToFront();
      if (globalPathLine && globalPathLine.bringToFront) globalPathLine.bringToFront();

      if (payload.start) {
        if (startMarker) pathLayer.removeLayer(startMarker);
        startMarker = L.circleMarker([payload.start.lat, payload.start.lon], { radius: 7 }).addTo(pathLayer);
        startMarker.bindTooltip('START', { permanent: false });
      }
      if (payload.goal) {
        if (goalMarker) pathLayer.removeLayer(goalMarker);
        goalMarker = L.circleMarker([payload.goal.lat, payload.goal.lon], { radius: 7 }).addTo(pathLayer);
        goalMarker.bindTooltip('GOAL', { permanent: false });
      }
    };

    // ---- WebChannel bridge ----
    let rosBridge = null;
    if (window.qt && window.qt.webChannelTransport) {
      new QWebChannel(qt.webChannelTransport, function(channel) {
        rosBridge = channel.objects.rosBridge;
      });
    }

    // ---- Buttons: Load Places / Load Obstacles / Save Obstacles ----
    const ToolsControl = L.Control.extend({
      options: { position: 'topleft' },
      onAdd: function() {
        const container = L.DomUtil.create('div', 'leaflet-bar');

        const btnPlaces = L.DomUtil.create('a', 'tool-btn', container);
        btnPlaces.href = '#';
        btnPlaces.title = 'Load places_to_go.csv';
        btnPlaces.innerHTML = 'Load Places';

        const btnObs = L.DomUtil.create('a', 'tool-btn', container);
        btnObs.href = '#';
        btnObs.title = 'Load permanent_obstacles_*.csv';
        btnObs.innerHTML = 'Load Obstacles';

        const btnSaveObs = L.DomUtil.create('a', 'tool-btn', container);
        btnSaveObs.href = '#';
        btnSaveObs.title = 'Publish + save permanent obstacles to CSV';
        btnSaveObs.innerHTML = 'Save Obstacles';

        const btnTemp = L.DomUtil.create('a', 'tool-btn', container);
        btnTemp.href = '#';
        btnTemp.title = 'Draw a temporary obstacle polygon (red)';
        btnTemp.innerHTML = 'Draw Temp Obstacle';

        const btnClearTemp = L.DomUtil.create('a', 'tool-btn', container);
        btnClearTemp.href = '#';
        btnClearTemp.title = 'Remove all temporary obstacles';
        btnClearTemp.innerHTML = 'Clear Temp Obstacles';

        L.DomEvent.disableClickPropagation(container);
        L.DomEvent.disableScrollPropagation(container);

        L.DomEvent.on(btnPlaces, 'click', function(e) {
          L.DomEvent.preventDefault(e);
          if (rosBridge && rosBridge.loadPlaces) rosBridge.loadPlaces();
        });
        L.DomEvent.on(btnObs, 'click', function(e) {
          L.DomEvent.preventDefault(e);
          if (rosBridge && rosBridge.loadPermanentObstacles) rosBridge.loadPermanentObstacles();
        });
        L.DomEvent.on(btnSaveObs, 'click', function(e) {
          L.DomEvent.preventDefault(e);
          if (rosBridge && rosBridge.savePermanentObstacles) {
            const payload = buildPermanentPayload();
            rosBridge.savePermanentObstacles(JSON.stringify(payload));
          }
        });

        L.DomEvent.on(btnTemp, 'click', function(e) {
          L.DomEvent.preventDefault(e);
          drawMode = 'temporary';
          const tool = new L.Draw.Polygon(map, { allowIntersection: false, showArea: true });
          tool.enable();
        });

        L.DomEvent.on(btnClearTemp, 'click', function(e) {
          L.DomEvent.preventDefault(e);
          clearTemporaryObstacles();
        });

        return container;
      }
    });
    map.addControl(new ToolsControl());

    // ---- Leaflet.draw toolbar (polygon only; NO lines) ----
    const drawControl = new L.Control.Draw({
      position: 'topright',
      draw: {
        polygon: { allowIntersection: false, showArea: true },
        polyline: false,
        rectangle: false,
        circle: false,
        marker: false,
        circlemarker: false
      },
      edit: { featureGroup: permObsLayer, remove: false } // we manage edits via points
    });
    map.addControl(drawControl);

    // Handle polygon creation (permanent vs temporary)
    map.on(L.Draw.Event.CREATED, function (event) {
      if (event.layerType !== 'polygon') return;

      const layer = event.layer;

      // TEMPORARY obstacle -> convert to editable red vertices
      if (drawMode === 'temporary') {
        drawMode = 'permanent';

        const tempId = String(nextTempObsId++);
        const ring = layer.getLatLngs()[0] || [];
        try { map.removeLayer(layer); } catch (e) {}

        ring.forEach(ll => addTempVertexMarker(tempId, ll.lat, ll.lng));
        rebuildTempObstaclePolygon(tempId);
        setSelectedTempObstacle(tempId);
        publishTemporaryObstacles();
        return;
      }

      // PERMANENT obstacle (existing behavior)
      const obsId = String(nextObstacleId++);
      const ring = layer.getLatLngs()[0] || [];
      ring.forEach(ll => addVertexMarker(obsId, ll.lat, ll.lng));

      try { map.removeLayer(layer); } catch (e) {}

      rebuildObstaclePolygon(obsId);
      setSelectedObstacle(obsId);
      });

  </script>
</body>
</html>
"""


def build_html() -> str:
    return (HTML_TEMPLATE
            .replace("__SW_LAT__", str(SW_LAT))
            .replace("__SW_LNG__", str(SW_LNG))
            .replace("__NE_LAT__", str(NE_LAT))
            .replace("__NE_LNG__", str(NE_LNG)))


def _pick_first(row: Dict[str, Any], keys: List[str]) -> str:
    for k in keys:
        if k in row and row[k] not in (None, ""):
            return row[k]
    return ""


def _norm_row(row: Dict[str, Any]) -> Dict[str, str]:
    return {(k or "").strip().lower(): (v or "").strip() for k, v in row.items()}


def load_places_to_go(csv_path: str) -> List[Dict[str, Any]]:
    places: List[Dict[str, Any]] = []
    if not os.path.exists(csv_path):
        return places

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f, skipinitialspace=True)
        for i, row in enumerate(reader):
            row = _norm_row(row)
            lat_s = _pick_first(row, ["lat", "latitude", "y"])
            lon_s = _pick_first(row, ["lng", "lon", "longitude", "x"])
            if not lat_s or not lon_s:
                continue
            try:
                lat = float(lat_s)
                lon = float(lon_s)
            except ValueError:
                continue

            name = _pick_first(row, [
                "name",
                "point name", "point_name", "pointname",
                "waypoint", "wp",
                "destination", "dest",
                "location", "loc",
                "label", "title", "tag", "marker", "id"
            ]) or f"Place {i+1}"

            places.append({"name": name, "lat": lat, "lon": lon})
    return places


def load_permanent_obstacles(csv_path: str) -> List[Dict[str, Any]]:
    obstacles_by_id: Dict[str, Dict[str, Any]] = {}
    if not csv_path or not os.path.exists(csv_path):
        return []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f, skipinitialspace=True)
        for row in reader:
            row = _norm_row(row)
            lat_s = _pick_first(row, ["lat", "latitude", "y"])
            lon_s = _pick_first(row, ["lng", "lon", "longitude", "x"])
            if not lat_s or not lon_s:
                continue
            try:
                lat = float(lat_s)
                lon = float(lon_s)
            except ValueError:
                continue

            oid = _pick_first(row, ["obstacle_id", "obstacle", "oid", "group_id", "id"]) or "0"
            label = _pick_first(row, ["label", "type", "category"]) or "permanent"

            if oid not in obstacles_by_id:
                obstacles_by_id[oid] = {"id": str(oid), "label": label, "points": []}
            obstacles_by_id[oid]["points"].append({"lat": lat, "lon": lon})

    def sort_key(item):
        try:
            return int(item["id"])
        except Exception:
            return item["id"]

    return sorted(obstacles_by_id.values(), key=sort_key)


class MapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RSX Rover Simulation Map (Leaflet + ROS2)")
        self.view = QWebEngineView()
        self.setCentralWidget(self.view)
        self.showMaximized()

        # ----- Points menu (Step 4) -----
        self.points_list = QListWidget()
        self.points_list.setSelectionMode(QListWidget.SingleSelection)

        self.lbl_start = QLabel("Start: (not set)")
        self.lbl_goal = QLabel("Goal: (not set)")

        self.btn_set_start = QPushButton("Set Selected As Start")
        self.btn_go = QPushButton("Go To Selected")

        box = QWidget()
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Points To Go To"))
        layout.addWidget(self.points_list)
        layout.addWidget(self.lbl_start)
        layout.addWidget(self.lbl_goal)

        row = QHBoxLayout()
        row.addWidget(self.btn_set_start)
        row.addWidget(self.btn_go)
        layout.addLayout(row)

        dock = QDockWidget("Points Menu", self)
        dock.setWidget(box)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)


class RosLeafletNode(Node):
    def __init__(self, window: MapWindow):
        super().__init__("leaflet_map_ros_ui")
        self.window = window
        self.view = window.view

        default_gui_dir = os.path.expanduser("~/rover_ws/src/rsx-rover/scripts/GUI")
        self.declare_parameter("gui_dir", default_gui_dir)
        self.gui_dir = os.path.expanduser(os.path.expandvars(
            self.get_parameter("gui_dir").get_parameter_value().string_value
        ))

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub_places = self.create_publisher(String, "/world/places_to_go", qos)
        self.pub_perm_obs = self.create_publisher(String, "/world/permanent_obstacles", qos)
        self.pub_temp_obs = self.create_publisher(String, "/world/temporary_obstacles", qos)

        # Mission pubs (Step 5)
        self.pub_start = self.create_publisher(String, "/mission/start", 10)
        self.pub_goal = self.create_publisher(String, "/mission/goal", 10)

        # Path display subscription (Step 5)
        self.create_subscription(String, "/planner/global_path", self.on_global_path, 10)
        self.create_subscription(String, "/planner/active_path", self.on_path, 10)
        self.create_subscription(String, "/sim/rover_pose", self.on_rover_pose, 10)
        self.create_subscription(String, "/perception/temporary_obstacles", self.on_perception, 10)
        self.create_subscription(String, "/planner/local_decision", self.on_local_decision, 10)

        self.create_subscription(String, "/world/places_to_go", self.on_places, 10)
        self.create_subscription(String, "/world/permanent_obstacles", self.on_perm_obs, 10)

        # In-memory place lookup: name -> (lat, lon)
        self.places: Dict[str, Tuple[float, float]] = {}
        self.start: Optional[Tuple[float, float]] = None
        self.goal: Optional[Tuple[float, float]] = None
        self.cur_pose: Optional[Tuple[float, float]] = None

        self.get_logger().info("UI ready. Use Load Places / Load Obstacles. Pick start + goal from the Points Menu.")
        self.get_logger().info("Planner will publish /planner/global_path which will be drawn automatically.")
        self.get_logger().info(f"CSV folder: {self.gui_dir}")

        # Hook up the menu buttons
        self.window.btn_set_start.clicked.connect(self._ui_set_start)
        self.window.btn_go.clicked.connect(self._ui_set_goal)

    def _call_js(self, fn_name: str, payload: dict):
        js = f"window.{fn_name}({json.dumps(payload)});"
        self.view.page().runJavaScript(js)

    def on_places(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad places JSON: {e}")
            return

        self._call_js("updatePlaces", payload)

        # Update the points menu list
        self.window.points_list.clear()
        self.places.clear()

        for p in payload.get("places", []):
            name = str(p.get("name", ""))
            lat = float(p.get("lat"))
            lon = float(p.get("lon"))
            self.places[name] = (lat, lon)

            item = QListWidgetItem(name)
            item.setData(Qt.UserRole, (lat, lon))
            self.window.points_list.addItem(item)

        # Auto-pick Start if it exists (name == "start" case-insensitive)
        start_name = None
        for n in self.places.keys():
            if n.strip().lower() == "start":
                start_name = n
                break
        if start_name is None and self.window.points_list.count() > 0:
            start_name = self.window.points_list.item(0).text()

        if start_name:
            self.start = self.places[start_name]
            self.window.lbl_start.setText(f"Start: {start_name}")
            self.pub_start.publish(String(data=json.dumps({
                "name": start_name, "lat": self.start[0], "lon": self.start[1]
            })))

    def on_perm_obs(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad obstacles JSON: {e}")
            return
        self._call_js("updatePermanentObstacles", payload)

    def on_global_path(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad global path JSON: {e}")
            return
        self._call_js("updateGlobalPath", payload)
    
    def on_path(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad path JSON: {e}")
            return
        self._call_js("updatePath", payload)

    def on_perception(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad perception JSON: {e}")
            return
        self._call_js("updatePerception", payload)

    def on_rover_pose(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad rover pose JSON: {e}")
            return

        # remember current pose for auto-start
        if "lat" in payload and "lon" in payload:
            try:
                self.cur_pose = (float(payload["lat"]), float(payload["lon"]))
            except Exception:
                pass

        self._call_js("updateRoverPose", payload)

    def on_local_decision(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad local decision JSON: {e}")
            return

        # Draw the visible wall segments (already lat/lon)
        self._call_js("updateDetectedWalls", payload)

    def _latest_file(self, pattern: str) -> str:
        matches = glob.glob(os.path.join(self.gui_dir, pattern))
        if not matches:
            return ""
        matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return matches[0]

    def load_places_from_csv(self):
        places_path = os.path.join(self.gui_dir, "places_to_go.csv")
        places = load_places_to_go(places_path)
        self.pub_places.publish(String(data=json.dumps({"places": places})))
        self.get_logger().info(f"Loaded+published places from CSV: {len(places)} places")

    def load_permanent_obstacles_from_csv(self):
        perm_obs_path = self._latest_file("permanent_obstacles_*.csv")
        perm_obs = load_permanent_obstacles(perm_obs_path) if perm_obs_path else []
        self.pub_perm_obs.publish(String(data=json.dumps({"obstacles": perm_obs})))
        self.get_logger().info(f"Loaded+published permanent obstacles from CSV: {len(perm_obs)} obstacles")

    def save_permanent_obstacles_payload(self, payload: dict):
        self.pub_perm_obs.publish(String(data=json.dumps(payload)))

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(self.gui_dir, f"permanent_obstacles_{ts}.csv")

        rows_written = 0
        now = time.time()

        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "timestamp", "lat", "lng", "label", "obstacle_id"])

            vid = 1
            for obs in payload.get("obstacles", []):
                oid = str(obs.get("id", "0"))
                label = str(obs.get("label", "permanent"))
                for pt in obs.get("points", []):
                    lat = pt.get("lat", None)
                    lon = pt.get("lon", None)
                    if lat is None or lon is None:
                        continue
                    w.writerow([vid, now, float(lat), float(lon), label, oid])
                    vid += 1
                    rows_written += 1

        self.get_logger().info(f"Saved permanent obstacles: {out_path} ({rows_written} vertices)")

    def publish_temporary_obstacles_payload(self, payload: dict):
        self.pub_temp_obs.publish(String(data=json.dumps(payload)))

    # ----- Points menu actions -----
    def _selected_point(self) -> Optional[Tuple[str, float, float]]:
        item = self.window.points_list.currentItem()
        if item is None:
            return None
        name = item.text()
        lat, lon = item.data(Qt.UserRole)
        return name, float(lat), float(lon)

    def _ui_set_start(self):
        sel = self._selected_point()
        if not sel:
            return
        name, lat, lon = sel
        self.start = (lat, lon)
        self.window.lbl_start.setText(f"Start: {name}")
        self.pub_start.publish(String(data=json.dumps({"name": name, "lat": lat, "lon": lon})))

    def _ui_set_goal(self):
        sel = self._selected_point()
        if not sel:
            return
        goal_name, goal_lat, goal_lon = sel

        # If start isn't set, auto-use the rover's current pose (best default)
        if self.start is None:
            if self.cur_pose is not None:
                s_lat, s_lon = self.cur_pose
                s_name = "Rover"
            elif "Start" in self.places:
                s_name = "Start"
                s_lat, s_lon = self.places["Start"]
            else:
                # fallback: use the first item in the list if it exists
                s_name, (s_lat, s_lon) = ("Rover", (goal_lat, goal_lon))

            self.start = (s_lat, s_lon)
            self.window.lbl_start.setText(f"Start: {s_name}")
            self.pub_start.publish(String(data=json.dumps({"name": s_name, "lat": s_lat, "lon": s_lon})))

        # Now publish goal
        self.goal = (goal_lat, goal_lon)
        self.window.lbl_goal.setText(f"Goal: {goal_name}")
        self.pub_goal.publish(String(data=json.dumps({"name": goal_name, "lat": goal_lat, "lon": goal_lon})))



class QtRosBridge(QObject):
    def __init__(self, ros_node: RosLeafletNode):
        super().__init__()
        self.ros_node = ros_node

    @pyqtSlot()
    def loadPlaces(self):
        self.ros_node.load_places_from_csv()

    @pyqtSlot()
    def loadPermanentObstacles(self):
        self.ros_node.load_permanent_obstacles_from_csv()

    @pyqtSlot(str)
    def savePermanentObstacles(self, payload_json: str):
        try:
            payload = json.loads(payload_json)
        except Exception:
            self.ros_node.get_logger().warn("Save Obstacles: payload was not valid JSON")
            return
        self.ros_node.save_permanent_obstacles_payload(payload)

    @pyqtSlot(str)
    def publishTemporaryObstacles(self, payload_json: str):
        try:
            payload = json.loads(payload_json)
        except Exception:
            self.ros_node.get_logger().warn("Temporary obstacles: payload was not valid JSON")
            return
        self.ros_node.publish_temporary_obstacles_payload(payload)

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    win = MapWindow()

    ui_node = RosLeafletNode(win)

    channel = QWebChannel()
    bridge = QtRosBridge(ui_node)
    channel.registerObject("rosBridge", bridge)
    win.view.page().setWebChannel(channel)

    win.view.setHtml(build_html(), baseUrl=QUrl("https://local/"))
    win.show()

    # Start other ROS nodes automatically
    global_planner = GlobalPlannerNode()
    detour_planner = LocalDetourPlannerNode()
    rover = RoverSimNode()
    fake_zed = FakeZedDetectorNode()

    executor = SingleThreadedExecutor()
    executor.add_node(ui_node)
    executor.add_node(global_planner)
    executor.add_node(detour_planner)
    executor.add_node(rover)
    executor.add_node(fake_zed)

    timer = QTimer()
    def _spin_all():
        executor.spin_once(timeout_sec=0.0)
    timer.timeout.connect(_spin_all)
    timer.start(20)

    def _shutdown():
        try:
            executor.remove_node(rover)
            executor.remove_node(global_planner)
            executor.remove_node(ui_node)
            executor.remove_node(fake_zed)
            executor.remove_node(detour_planner)
        except Exception:
            pass

        rover.destroy_node()
        global_planner.destroy_node()
        detour_planner.destroy_node()
        ui_node.destroy_node()
        fake_zed.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(_shutdown)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
