import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QFileDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage
from PIL import Image
import matplotlib.pyplot as plt


class PhotoViewerApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Photo Viewer")
        self.setGeometry(100, 100, 800, 700)

        self.folder_path = None
        self.photos = []
        self.current_photo_idx = 0
        self.image_width = 0
        self.image_height = 0

        # Layouts
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # Open Folder button
        self.open_button = QPushButton("Open Folder")
        self.open_button.clicked.connect(self.open_folder)

        # Canvas to display the main image
        self.canvas = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.canvas.setScene(self.scene)

        # Mouse coordinates and greyscale label
        self.coords_label = QLabel("Mouse Coordinates: (x, y)\nImage Dimensions: (width, height)", self)

        # Change Photo button
        self.change_button = QPushButton("Toggle Photo")
        self.change_button.clicked.connect(self.change_photo)

        # Add widgets to the main layout
        self.main_layout.addWidget(self.open_button)
        self.main_layout.addWidget(self.canvas)
        self.main_layout.addWidget(self.coords_label)
        self.main_layout.addWidget(self.change_button)

        # Mouse move and click events
        self.canvas.setMouseTracking(True)
        self.canvas.mouseMoveEvent = self.show_coordinates
        self.canvas.mousePressEvent = self.show_greyscale_value

    def open_folder(self):
        """Open the folder and load 12 photos."""
        folder = QFileDialog.getExistingDirectory(self, "Open Folder")
        if folder:
            self.folder_path = folder
            self.load_photos()
            self.display_photo()

    def load_photos(self):
        """Load exactly 12 photos from the selected folder."""
        self.photos.clear()
        for filename in os.listdir(self.folder_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):  # Check for image files
                img_path = os.path.join(self.folder_path, filename)
                img = Image.open(img_path)
                self.photos.append(img)

        # Ensure there are exactly 12 photos
        if len(self.photos) != 12:
            print("Please select a folder containing exactly 12 photos.")
            self.folder_path = None
            return

    def display_photo(self):
        """Display the current main photo on the canvas."""
        photo = self.photos[self.current_photo_idx]
        pixmap = QPixmap.fromImage(self.pil_to_qimage(photo))
        item = QGraphicsPixmapItem(pixmap)
        self.scene.clear()  # Clear previous photo
        self.scene.addItem(item)
        self.canvas.setScene(self.scene)

        # Store image dimensions
        self.image_width = photo.width
        self.image_height = photo.height

        # Update the label with the image dimensions
        self.coords_label.setText(f"Mouse Coordinates: (x, y)\nImage Dimensions: ({self.image_width}, {self.image_height})")

    def pil_to_qimage(self, pil_image):
        """Convert PIL image to QImage."""
        rgb_image = pil_image.convert("RGB")
        data = rgb_image.tobytes("raw", "RGB")
        return QImage(data, rgb_image.width, rgb_image.height, QImage.Format_RGB888)

    def change_photo(self):
        """Change to the next photo."""
        self.current_photo_idx = (self.current_photo_idx + 1) % 12
        self.display_photo()

    def show_coordinates(self, event):
        """Display mouse coordinates and greyscale values for all 12 photos on hover."""
        # Get the mouse position in the view
        view_pos = event.pos()
        
        # Transform the view position to scene coordinates
        scene_pos = self.canvas.mapToScene(view_pos)
        
        # Get the pixmap item in the scene (the image)
        item = self.scene.items()[0]
        
        # Get the image's transformation matrix (used for scaling/translation)
        transform = item.transform()
        
        # Apply the transformation to the mouse position
        image_pos = transform.inverted()[0].map(scene_pos)

        # Convert to integer coordinates for pixel access
        x, y = int(image_pos.x()), int(image_pos.y())
        
        # Ensure the coordinates are within the bounds of the image
        if 0 <= x < item.pixmap().width() and 0 <= y < item.pixmap().height():
            self.coords_label.setText(f"Mouse Coordinates: ({x}, {y})\nImage Dimensions: ({self.image_width}, {self.image_height})")
            
            # Calculate greyscale values for all photos at the hovered pixel
            greyscale_values = []
            for photo in self.photos:
                pixel = photo.convert("L").getpixel((x, y))  # Convert to greyscale and get the pixel value
                greyscale_values.append(pixel)
            
            # Display the greyscale values for all 12 photos
            greyscale_text = "Greyscale Values: " + ", ".join([str(value) for value in greyscale_values])
            self.coords_label.setText(f"{self.coords_label.text()}\n{greyscale_text}")

    def show_greyscale_value(self, event):
        """Plot the greyscale values for all photos at the clicked pixel."""
        # Get the mouse click position in the view (canvas coordinates)
        view_pos = event.pos()
        
        # Transform the canvas coordinates to scene coordinates
        scene_pos = self.canvas.mapToScene(view_pos)
        
        # Get the pixmap item in the scene (the image)
        item = self.scene.items()[0]
        
        # Get the image's transformation matrix (used for scaling/translation)
        transform = item.transform()
        
        # Apply the transformation to the mouse position
        image_pos = transform.inverted()[0].map(scene_pos)
        
        # Convert to integer coordinates for pixel access
        x, y = int(image_pos.x()), int(image_pos.y())

        # Ensure the coordinates are within the bounds of the image
        if 0 <= x < item.pixmap().width() and 0 <= y < item.pixmap().height():
            greyscale_values = []
            
            # Get the greyscale values for all photos at the clicked pixel
            for photo in self.photos:
                pixel = photo.convert("L").getpixel((x, y))  # Convert to greyscale and get the pixel value
                greyscale_values.append(pixel)
            
            # Plot the greyscale values for the 12 images
            self.plot_greyscale_values(greyscale_values)

    def plot_greyscale_values(self, greyscale_values):
        """Plot the greyscale values on a line plot with fixed y-axis range."""
        plt.figure(figsize=(10, 6))
        plt.plot(range(1, 13), greyscale_values, marker='o', linestyle='-', color='b')

        # Set fixed y-axis limits
        plt.ylim(0, 255)

        # Title and labels with better formatting
        plt.title("Greyscale Values at Clicked Pixel", fontsize=14, fontweight='bold')
        plt.xlabel("Photo Index (Filters)", fontsize=12)
        plt.ylabel("Greyscale Value (0-255)", fontsize=12)
        
        # Set x-ticks and y-ticks with labels
        plt.xticks(range(1, 13), [f"Filter {i}" for i in range(1, 13)], rotation=45)
        plt.yticks(range(0, 256, 51), [str(i) for i in range(0, 256, 51)])
        
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = PhotoViewerApp()
    viewer.show()
    sys.exit(app.exec_())
