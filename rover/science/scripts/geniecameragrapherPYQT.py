import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QFileDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage
from PIL import Image
import matplotlib.pyplot as plt


class PhotoViewerApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Photo Viewer")
        self.setGeometry(100, 100, 1000, 900)  # Increased window size for a larger image display

        self.folder_path = None
        self.photos = []
        self.current_photo_idx = 0

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
        self.coords_label = QLabel("Mouse Coordinates: (x, y)\nGreyscale Values:", self)

        # Add widgets to the main layout
        self.main_layout.addWidget(self.open_button)
        self.main_layout.addWidget(self.canvas)
        self.main_layout.addWidget(self.coords_label)

        # Mouse move and click events
        self.canvas.setMouseTracking(True)
        self.canvas.mouseMoveEvent = self.show_coordinates
        self.canvas.mousePressEvent = self.show_greyscale_value


    def open_folder(self):
        """Open the folder and load photos."""
        folder = QFileDialog.getExistingDirectory(self, "Open Folder")
        if folder:
            self.folder_path = folder
            self.load_photos()
            self.display_photo()


    def load_photos(self):
        """Load photos from the selected folder."""
        self.photos.clear()
        for filename in os.listdir(self.folder_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):  # Add more formats if needed
                img_path = os.path.join(self.folder_path, filename)
                try:
                    img = Image.open(img_path)
                    self.photos.append(img)
                except Exception as e:
                    print(f"Error loading image {filename}: {e}")

        # Ensure there are exactly 12 photos
        if len(self.photos) != 12:
            print("Please select a folder containing exactly 12 photos.")
            self.folder_path = None
            return


    def display_photo(self):
        """Display the current main photo on the canvas."""
        if not self.photos:
            print("No photos loaded.")  # Debug: No photos loaded
            return
        
        photo = self.photos[self.current_photo_idx]
        print(f"Displaying photo {self.current_photo_idx + 1}")  # Debug: Show which photo is being displayed

        # Resize the photo to make it 1000x1000
        self.resized_photo = photo.resize((1000, 1000), Image.Resampling.LANCZOS)  # Updated to 1000x1000

        # Convert the resized photo to QPixmap
        pixmap = QPixmap.fromImage(self.pil_to_qimage(self.resized_photo))

        # Create a QGraphicsPixmapItem for the resized image
        item = QGraphicsPixmapItem(pixmap)
        
        # Clear previous photo and add the new item
        self.scene.clear()
        self.scene.addItem(item)
        self.canvas.setScene(self.scene)

        # Update the image dimensions
        self.image_width = self.resized_photo.width
        self.image_height = self.resized_photo.height

        # Update the label with the image dimensions and current photo info
        self.coords_label.setText(f"Mouse Coordinates: (x, y)\nImage Dimensions: ({self.image_width}, {self.image_height})\n"
                                  f"Displaying Photo {self.current_photo_idx + 1} of 12")

        print(f"Photo dimensions: {self.image_width} x {self.image_height}")  # Debug: Check photo size


    def pil_to_qimage(self, pil_image):
        """Convert PIL image to QImage."""
        rgb_image = pil_image.convert("RGB")
        data = rgb_image.tobytes("raw", "RGB")
        return QImage(data, rgb_image.width, rgb_image.height, QImage.Format_RGB888)


    def show_coordinates(self, event):
        """Display mouse coordinates and greyscale values on hover."""
        # Ensure the scene has at least one item (the image)
        items = self.scene.items()
        if not items:
            print("No items in the scene.")
            return  # Exit early if the scene is empty
        
        view_pos = event.pos()
        scene_pos = self.canvas.mapToScene(view_pos)
        
        # Get the image's pixmap item
        item = items[0]
        
        # Apply the transformation to get the coordinates in the image
        transform = item.transform()
        image_pos = transform.inverted()[0].map(scene_pos)

        x, y = int(image_pos.x()), int(image_pos.y())
        
        # Ensure the coordinates are within the image bounds
        if 0 <= x < self.resized_photo.width and 0 <= y < self.resized_photo.height:
            self.coords_label.setText(f"Mouse Coordinates: ({x}, {y})")

            # Calculate greyscale values for all photos
            greyscale_values = []
            for photo in self.photos:
                pixel = photo.convert("L").getpixel((x, y))  # Get greyscale value from original image
                greyscale_values.append(pixel)
            
            greyscale_text = "Greyscale Values: " + ", ".join([str(value) for value in greyscale_values])
            self.coords_label.setText(f"{self.coords_label.text()}\n{greyscale_text}")


    def show_greyscale_value(self, event):
        """Plot the greyscale values for all photos at the clicked pixel."""
        view_pos = event.pos()
        scene_pos = self.canvas.mapToScene(view_pos)
        
        # Ensure the scene has at least one item (the image)
        items = self.scene.items()
        if not items:
            return  # No item in the scene, so return without further processing
        
        # Get the image's pixmap item
        item = items[0]
        
        # Apply the transformation to get the coordinates in the image
        transform = item.transform()
        image_pos = transform.inverted()[0].map(scene_pos)
        
        x, y = int(image_pos.x()), int(image_pos.y())

        if 0 <= x < self.resized_photo.width and 0 <= y < self.resized_photo.height:
            greyscale_values = []
            for photo in self.photos:
                pixel = photo.convert("L").getpixel((x, y))  # Get greyscale value from original image
                greyscale_values.append(pixel)
            
            # Plot the greyscale values
            self.plot_greyscale_values(greyscale_values)


    def plot_greyscale_values(self, greyscale_values):
        """Plot the greyscale values for all photos."""
        plt.figure(figsize=(10, 6))
        plt.plot(range(1, 13), greyscale_values, marker='o', linestyle='-', color='b')
        plt.ylim(0, 255)
        plt.title("Greyscale Values at Clicked Pixel", fontsize=14, fontweight='bold')
        plt.xlabel("Photo Index", fontsize=12)
        plt.ylabel("Greyscale Value (0-255)", fontsize=12)
        plt.xticks(range(1, 13), [f"Photo {i}" for i in range(1, 13)], rotation=45)
        plt.yticks(range(0, 256, 51), [str(i) for i in range(0, 256, 51)])
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = PhotoViewerApp()
    viewer.show()
    sys.exit(app.exec_())


