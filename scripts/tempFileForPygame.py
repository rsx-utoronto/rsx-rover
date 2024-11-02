import sys
import pygame
import json
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPainter
widgetWidth = 400
widgetHeight = 300

class PygameOverlay(QWidget):
    def __init__(self, map_image_path, metadata_path):
        super().__init__()
        self.setMinimumSize(widgetWidth, widgetHeight)  # Set minimum size for the widget
        pygame.init()
        self.surface = pygame.Surface((widgetWidth, widgetHeight))  # Initialize Pygame surface
        
        # Load the map image
        self.map_image = pygame.image.load(map_image_path)
        print(f"Map image size: {self.map_image.get_size()}")  # Print image size
        self.metadata = self.load_metadata(metadata_path)

        # Calculate scale and offsets for GPS coordinates
        self.latitude_min = self.metadata['bounds']['southwest']['lat']
        self.latitude_max = self.metadata['bounds']['northeast']['lat']
        self.longitude_min = self.metadata['bounds']['southwest']['lng']
        self.longitude_max = self.metadata['bounds']['northeast']['lng']
        self.image_width = self.map_image.get_width()
        self.image_height = self.map_image.get_height()

    def load_metadata(self, path):
        with open(path, 'r') as f:
            return json.load(f)

    def gps_to_pixel(self, latitude, longitude):
        # Convert GPS coordinates to pixel coordinates
        x = int((longitude - self.longitude_min) / (self.longitude_max - self.longitude_min) * widgetWidth)
        y = int((1 - (latitude - self.latitude_min) / (self.latitude_max - self.latitude_min)) * widgetHeight)

        print(f"Converted GPS ({latitude}, {longitude}) to pixel ({x}, {y})")  # Debugging output
        return x, y

    def paintEvent(self, event):
        # Render the map and GPS points
        self.render_map_and_gps()
        
        # Convert the Pygame surface to an RGB array
        frame = pygame.surfarray.array3d(self.surface)  # Get the surface as an array
        frame = np.rot90(frame)  # Rotate to make the axes match
        frame = np.flipud(frame)  # Flip vertically to correct orientation
        height, width, _ = frame.shape

        # Create a QImage from the numpy array, converting to bytes
        img = QImage(frame.tobytes(), width, height, QImage.Format_RGB888)

        # Create a QPainter to draw the image on the widget
        painter = QPainter(self)
        painter.drawImage(0, 0, img)

    def render_map_and_gps(self):
        # Clear the surface
        self.surface.fill((255, 255, 255))  # Fill with white
        
        # Draw the map image
        self.surface.blit(self.map_image, (0, 0))
        
        # Example GPS coordinate to mark (replace with actual data)
        gps_coordinate = (40.7, -74)  # Example: New York City
        gps_x, gps_y = self.gps_to_pixel(*gps_coordinate)

        # Check if pixel coordinates are within the bounds of the image
        if 0 <= gps_x < self.image_width and 0 <= gps_y < self.image_height:
            # Draw the GPS point
            pygame.draw.circle(self.surface, (0, 255, 0), (gps_x, gps_y), 5)  # Draw a green circle at GPS point
        else:
            print(f"GPS point out of bounds: ({gps_x}, {gps_y})")  # Debugging output

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Map Overlay Example")

        # Create layout
        layout = QVBoxLayout()

        # Create an instance of PygameOverlay
        self.map_overlay = PygameOverlay("/home/laura/rover_ws/src/rsx-rover/map_output/map_screenshot.png", 
                                          "/home/laura/rover_ws/src/rsx-rover/map_output/map_metadata.json")
        
        # Add PygameOverlay to layout
        layout.addWidget(self.map_overlay)

        # Add other widgets as needed
        label = QLabel("GPS Map Overlay")
        layout.addWidget(label)

        # Create a central widget and set the layout
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())
