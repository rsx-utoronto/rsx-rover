#!/usr/bin/env python3
import rospy
import random
import os
import sys
from sensor_msgs.msg import NavSatFix
from PyQt5.QtCore import QThread, QUrl,QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtWebEngineWidgets import QWebEngineView
import json
import rospkg 

class GPSVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node('gps_visualizer_node', anonymous=True)
        self.setWindowTitle("3D Rover GPS Visualization (Google Earth Style)")
        self.setGeometry(100, 100, 800, 600)

        # Set up layout for the window
        layout = QVBoxLayout()

        # Set up the web view for CesiumJS
        self.map_view = QWebEngineView(self)
        self.map_view.setGeometry(50, 50, 700, 500)
        self.load_map()

        layout.addWidget(QLabel("Rover Location on 3D Map (Google Earth Style)"))
        layout.addWidget(self.map_view)
        self.setLayout(layout)

        # ROS subscriber for GPS data
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)

        # Timer for updating the map in real time
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(1000)  # Update every 1 second

        # Path list to store GPS coordinates
        self.path = []

    def load_map(self):
        """
        Initialize and load the CesiumJS map dynamically
        """
        print("Loading map...")  # This will confirm if the method is called

        cesium_html = '''
        <!DOCTYPE html>
        <html>
        <head>
            <script src="file:///home/definitely_not_marvin/Downloads/CesiumJS/Build/Cesium/Cesium.js"></script>
            <link href="file:///home/definitely_not_marvin/Downloads/CesiumJS/Build/Cesium/Widgets/widgets.css" rel="stylesheet">
            <style>
                body, html { margin: 0; padding: 0; width: 100%; height: 100%; }
                #cesiumContainer { width: 100%; height: 100%; }
            </style>
        </head>
        <body>
            <div id="cesiumContainer"></div>
            <script>
                var viewer = new Cesium.Viewer('cesiumContainer', {
                    terrainProvider: Cesium.createWorldTerrain()
                });

                var roverPath = [];  // Stores the rover path

                function updateRoverPosition(lat, lon, alt) {
                    var roverLocation = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
                    if (roverPath.length === 0) {
                        var roverEntity = viewer.entities.add({
                            position: roverLocation,
                            point: { pixelSize: 10, color: Cesium.Color.RED }
                        });
                    } else {
                        roverPath.push(roverLocation);
                        viewer.entities.add({
                            polyline: {
                                positions: roverPath,
                                width: 5,
                                material: Cesium.Color.RED
                            }
                        });
                    }
                    viewer.zoomTo(viewer.entities);
                }

                function receiveGPSData(gpsData) {
                    var data = JSON.parse(gpsData);
                    updateRoverPosition(data.latitude, data.longitude, data.altitude);
                }
            </script>
        </body>
        </html>
        '''

        # Use rospack to get the path to the current package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('rover')  # Replace 'rover' with your package name
        scripts_path = os.path.join(package_path, 'scripts')
        
        # Ensure the directory exists
        if not os.path.exists(scripts_path):
            os.makedirs(scripts_path)

        # Save the HTML map file in the package's 'scripts' directory
        html_file_path = os.path.join(scripts_path, 'cesium_map.html')

        # Write the HTML to the file
        with open(html_file_path, 'w') as f:
            f.write(cesium_html)

        print(f"Map file created at: {html_file_path}")  # Debugging line
        self.map_view.setUrl(QUrl.fromLocalFile(html_file_path))

    def gps_callback(self, msg):
        rospy.loginfo(f"Received GPS Data: {msg.latitude}, {msg.longitude}, {msg.altitude}")
        # Send the data to the CesiumJS map
        gps_data = json.dumps({
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        })
        rospy.loginfo(f"Sending to CesiumJS: {gps_data}")  # Add this line to log the GPS data being passed
        self.map_view.page().runJavaScript(f"receiveGPSData('{gps_data}');")


    def update_map(self):
        # Called periodically to refresh the map with the rover's latest location
        pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gps_visualizer = GPSVisualizer()
    gps_visualizer.show()
    sys.exit(app.exec_())  # Start the event loop
