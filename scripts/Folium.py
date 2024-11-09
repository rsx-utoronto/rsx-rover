#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import folium
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class GpsDataRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, node, *args, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        map_content = self.node.generate_map()
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(map_content.encode())


class GPSDataSubscriber:
    def __init__(self):
        rospy.init_node("gps_data_subscriber")
        self.history = []  # List to store GPS coordinates

        self.subscription = rospy.Subscriber("/gps/fix", NavSatFix, self.callback)

        
        # Start the HTTP server in a separate thread
        http_server_thread = threading.Thread(target=self.start_server)
        http_server_thread.daemon = True
        http_server_thread.start()

    def callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        if latitude != 0.0 and longitude != 0.0:  # Filter out invalid GPS data
            self.history.append((latitude, longitude))  # Append GPS coordinates

    def start_server(self):
        server_address = ("", 8080)
        httpd = HTTPServer(
            server_address,
            lambda *args, **kwargs: GpsDataRequestHandler(self, *args, **kwargs),
        )
        print("Starting visualizer at http://localhost:8080")

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            httpd.server_close()

    def generate_map(self):
        # Create a Folium map centered on the first GPS coordinate if available
        if len(self.history) > 0:
            initial_latitude, initial_longitude = self.history[0]
            map = folium.Map(location=[initial_latitude, initial_longitude], zoom_start=15)

            # Draw the path taken by the rover
            folium.PolyLine(locations=self.history, color='blue', weight=5).add_to(map)

            # Add markers for each GPS point
            for lat, lon in self.history:
                folium.Marker([lat, lon], icon=folium.Icon(icon="cloud", color="blue")).add_to(map)
        else:
            # Default map if no history
            map = folium.Map(location=[0, 0], zoom_start=2)

        # Save the map to an HTML string
        map_html = map.get_root().render()
        return f"<!DOCTYPE html><html><head><title>GPS Location</title></head><body>{map_html}</body></html>"


if __name__ == "__main__":
    gps_data_subscriber = GPSDataSubscriber()
    
    try:
        rospy.spin()  # Keep the node running
    except KeyboardInterrupt:
        print("Shutting down...")
