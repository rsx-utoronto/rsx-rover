import sys
import pygame
import json
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QImage, QPainter

class PygameOverlay(QWidget):
    def __init__(self, map_image_path, metadata_path):
        super().__init__()
        self.setMinimumSize(800, 600)

        # Initialize Pygame
        pygame.init()
        self.surface = pygame.Surface((800, 600))

        # Load the map image
        self.map_image = pygame.image.load(map_image_path)
        self.metadata = self.load_metadata(metadata_path)

        # Calculate scale and offsets for GPS coordinates
        self.latitude_min = self.metadata['bounds']['southwest']['lat']
        self.latitude_max = self.metadata['bounds']['northeast']['lat']
        self.longitude_min = self.metadata['bounds']['northeast']['lng']
        self.longitude_max = self.metadata['bounds']['southwest']['lng']
        self.image_width = self.map_image.get_width()
        self.image_height = self.map_image.get_height()

    def load_metadata(self, path):
        with open(path, 'r') as f:
            return json.load(f)

    def gps_to_pixel(self, latitude, longitude):
        # Convert GPS coordinates to pixel coordinates
        x = int((longitude - self.longitude_min) / (self.longitude_max - self.longitude_min) * self.image_width)
        y = int((1 - (latitude - self.latitude_min) / (self.latitude_max - self.latitude_min)) * self.image_height)
        return x, y

    def paintEvent(self, event):
        # Render the map and GPS points
        self.render_map_and_gps()

        # Convert the Pygame surface to a format PyQt can use
        frame = pygame.surfarray.array3d(self.surface)
        height, width, _ = frame.shape
        frame = frame.swapaxes(0, 1)  # Switch axes to fit PyQt
        img = QImage(frame, width, height, QImage.Format_RGB888)

        # Create a QPainter to draw the image on the widget
        painter = QPainter(self)
        painter.drawImage(0, 0, img)

    def render_map_and_gps(self):
        # Clear the surface
        self.surface.fill((255, 255, 255))  # Fill with white

        # Draw the map image
        self.surface.blit(self.map_image, (0, 0))

        # Example GPS coordinate to mark (replace with actual data)
        gps_coordinate = (37.7749, -122.4194)  # Example: San Francisco
        gps_x, gps_y = self.gps_to_pixel(*gps_coordinate)

        # Draw the GPS point
        pygame.draw.circle(self.surface, (0, 255, 0), (gps_x, gps_y), 5)  # Draw GPS point

class MainWindow(QMainWindow):
    def __init__(self, map_image_path, metadata_path):
        super().__init__()
        self.setWindowTitle("GPS Overlay with Pygame")
        self.setCentralWidget(PygameOverlay(map_image_path, metadata_path))

if __name__ == "__main__":
    map_image_path = "/home/laura/rover_ws/src/rsx-rover/map_output/map_screenshot.png"  # Specify the path to your map image
    metadata_path = "/home/laura/rover_ws/src/rsx-rover/map_output/map_metadata.json"   # Specify the path to your metadata file
    app = QApplication(sys.argv)
    window = MainWindow(map_image_path, metadata_path)
    window.show()
    sys.exit(app.exec_())
