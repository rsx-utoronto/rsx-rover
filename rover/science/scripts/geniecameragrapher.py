import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import os
import matplotlib.pyplot as plt
import cv2

class PhotoViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Photo Viewer")
        self.root.geometry("800x700")
        
        self.folder_path = None
        self.photos = []
        self.photo_names = []
        self.current_photo_idx = 0
        
        self.relative_spectral_sensitivity = {
            "440": 0.78,
            "500": 0.97,
            "530": 1.0,
            "570": 0.96,
            "610": 0.9,
            "670": 0.76,
            "740": 0.58,
            "780": 0.44,
            "840": 0.31,
            "900": 0.18,
            "950": 0.1,
            "1000": 0.05
        }

        # Create the open folder button
        self.open_button = tk.Button(self.root, text="Open Folder", command=self.open_folder)
        self.open_button.pack()
        
        # Canvas to display the main image - increase size
        self.canvas = tk.Canvas(self.root, width=700, height=500)
        self.canvas.pack(pady=10)
        
        # Label to display mouse coordinates on the main image
        self.coords_label = tk.Label(self.root, text="Mouse Coordinates: (x, y)\nGreyscale Values:", font=("Arial", 12))
        self.coords_label.pack()

        # Create a frame to hold the smaller images
        self.thumb_frame = tk.Frame(self.root)
        self.thumb_frame.pack(pady=10)

        # Button to change the photo
        self.change_button = tk.Button(self.root, text="Toggle Photo", command=self.change_photo)
        self.change_button.pack(pady=10)

        # Bind the canvas mouse movement to display coordinates
        self.canvas.bind("<Motion>", self.show_coordinates)
        self.canvas.bind("<Button-1>", self.show_greyscale_value)

        self.wavelength = [440, 500, 530, 570, 610, 670, 740, 780, 840, 900, 950, 1000]
        self.flat_norm = cv2.imread("../genie_calibration_data/flat_norm")
        self.dark_corr = cv2.imread("../genie_calibration_data/dark_corr")

        with open("../genie_calibration_data/calibration_constants.csv", "r") as f:
            lines = f.readlines
            self.M = {
                "440": 0.78,
                "500": 0.97,
                "530": 1.0,
                "570": 0.96,
                "610": 0.9,
                "670": 0.76,
                "740": 0.58,
                "780": 0.44,
                "840": 0.31,
                "900": 0.18,
                "950": 0.1,
                "1000": 0.05
            }
            self.C = {
                "440": 0.78,
                "500": 0.97,
                "530": 1.0,
                "570": 0.96,
                "610": 0.9,
                "670": 0.76,
                "740": 0.58,
                "780": 0.44,
                "840": 0.31,
                "900": 0.18,
                "950": 0.1,
                "1000": 0.05
            }
            for line in lines:
                l = line.split(",")
                self.M[l[0]] = float(l[1])
                self.C[l[0]] = float(l[2])
        
    def open_folder(self):
        """Open the folder and load 12 photos."""
        folder = filedialog.askdirectory()
        if folder:
            self.folder_path = folder
            self.load_photos()
            self.display_photo()
            self.display_thumbnails()

    def load_photos(self):
        """Load exactly 12 photos from the selected folder."""
        self.photos.clear()
        for filename in os.listdir(self.folder_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):
                img_path = os.path.join(self.folder_path, filename)
                img = Image.open(img_path)
                self.photos.append(img)
                self.photo_names.append(int(filename.split('.')[0].split("_")[2].split("nm")[0]))  # Store the name without extension

        # Ensure there are exactly 12 photos
        if len(self.photos) != 12:
            tk.messagebox.showerror("Error", "Please select a folder containing exactly 12 photos.")
            self.folder_path = None
            return

        temp = self.photos[0]
        temp_name = self.photo_names[0]
        self.photos[0:11] = self.photos[1:12] 
        self.photos[11] = temp
        self.photo_names[0:11] = self.photo_names[1:12]
        self.photo_names[11] = temp_name

    def display_photo(self):
        """Display the current main photo on the canvas, resized to fit the canvas."""
        photo = self.photos[self.current_photo_idx]
        
        # Get canvas dimensions
        canvas_width = self.canvas.winfo_width() or 700  # Default if not yet rendered
        canvas_height = self.canvas.winfo_height() or 500
        
        # Calculate resize ratio while preserving aspect ratio
        img_width, img_height = photo.size
        width_ratio = canvas_width / img_width
        height_ratio = canvas_height / img_height
        ratio = min(width_ratio, height_ratio)
        
        # Resize image to fit canvas
        new_width = int(img_width * ratio)
        new_height = int(img_height * ratio)
        resized_photo = photo.resize((new_width, new_height), Image.LANCZOS)
        
        self.main_photo = ImageTk.PhotoImage(resized_photo)
        
        # Clear previous image and center the new one
        self.canvas.delete("all")
        x_center = (canvas_width - new_width) // 2
        y_center = (canvas_height - new_height) // 2
        self.canvas.create_image(x_center, y_center, anchor=tk.NW, image=self.main_photo)
        
        # Store original and resized dimensions for coordinate mapping
        self.original_dims = (img_width, img_height)
        self.resized_dims = (new_width, new_height)
        self.img_offset = (x_center, y_center)
    
    def display_thumbnails(self):
        """Display smaller versions of each photo below the main photo with numbers."""
        for widget in self.thumb_frame.winfo_children():
            widget.destroy()

        for i, photo in enumerate(self.photos):
            thumb = photo.resize((50, 50), Image.LANCZOS)
            thumb_photo = ImageTk.PhotoImage(thumb)
            
            # Create a button for the thumbnail
            button = tk.Button(self.thumb_frame, image=thumb_photo, command=lambda i=i: self.set_main_photo(i))
            button.image = thumb_photo  # Keep a reference to avoid garbage collection
            
            # Create a label with the number
            label = tk.Label(self.thumb_frame, text=str(i+1), font=("Arial", 10))
            
            # Place the button and label in a grid
            button.grid(row=0, column=i)
            label.grid(row=1, column=i)

    def set_main_photo(self, idx):
        """Set the main photo based on the selected thumbnail."""
        self.current_photo_idx = idx
        self.display_photo()

    def change_photo(self):
        """Change to the next photo."""
        self.current_photo_idx = (self.current_photo_idx + 1) % 12
        self.display_photo()

    def show_coordinates(self, event):
        """Display mouse coordinates and greyscale values for all 12 photos on hover."""
        x, y = event.x, event.y
        self.coords_label.config(text=f"Mouse Coordinates: ({x}, {y})")
        
        # Calculate greyscale values for all photos at the hovered pixel
        greyscale_values = []
        for photo in self.photos:
            pixel = photo.convert("L").getpixel((x, y))  # Convert to greyscale and get the pixel value
            greyscale_values.append(pixel)
        
        # Display the greyscale values for all 12 photos
        greyscale_text = "Greyscale Values: " + ", ".join([str(value) for value in greyscale_values])
        self.coords_label.config(text=f"{self.coords_label.cget('text')}\n{greyscale_text}")


    def show_greyscale_value(self, event):
        """Plot the greyscale values for all photos at the clicked pixel."""
        x, y = event.x, event.y
        greyscale_values = []
        
        i = 0
        for photo in self.photos:
            print(self.photo_names[i])
            pixel = photo.convert("L").getpixel((x, y))  # Convert to greyscale and get the pixel value
            pixel = float(pixel / self.relative_spectral_sensitivity[str(self.photo_names[i])])
            greyscale_values.append(pixel)
            i += 1

        # Plot the greyscale values for the 12 images
        self.plot_greyscale_values(greyscale_values)

    def plot_greyscale_values(self, greyscale_values):
        """Plot the greyscale values on a line plot with fixed y-axis range."""
        plt.figure(figsize=(10, 6))
        plt.plot(self.photo_names, greyscale_values, marker='o', linestyle='-', color='b')

        # Set fixed y-axis limits
        plt.ylim(0, 255)

        # Title and labels with better formatting
        plt.title("Greyscale Values at Clicked Pixel", fontsize=14, fontweight='bold')
        plt.xlabel("Photo Index", fontsize=12)
        plt.ylabel("Greyscale Value (0-255)", fontsize=12)
        
        # Set x-ticks and y-ticks with labels
        plt.xticks(self.wavelength, ["440", "500", "530", "570", "610", "670", "740", "780", "840", "900", "950", "1000"], rotation=45)
        plt.yticks(range(0, 256, 51), [str(i) for i in range(0, 256, 51)])
        
        plt.grid(True)
        plt.tight_layout()
        plt.show()




if __name__ == "__main__":
    root = tk.Tk()
    app = PhotoViewerApp(root)
    root.mainloop()
