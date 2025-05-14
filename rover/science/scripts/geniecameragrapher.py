import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import os
import matplotlib.pyplot as plt

class PhotoViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Photo Viewer")
        self.root.geometry("800x700")
        
        self.folder_path = None
        self.photos = []
        self.current_photo_idx = 0
        
        # Create the open folder button
        self.open_button = tk.Button(self.root, text="Open Folder", command=self.open_folder)
        self.open_button.pack()
        
        # Canvas to display the main image
        self.canvas = tk.Canvas(self.root, width=500, height=400)
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

        # Ensure there are exactly 12 photos
        if len(self.photos) != 12:
            tk.messagebox.showerror("Error", "Please select a folder containing exactly 12 photos.")
            self.folder_path = None
            return

    def display_photo(self):
        """Display the current main photo on the canvas."""
        photo = self.photos[self.current_photo_idx]
        self.main_photo = ImageTk.PhotoImage(photo)
        
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.main_photo)
    
    def display_thumbnails(self):
        """Display smaller versions of each photo below the main photo with numbers."""
        for widget in self.thumb_frame.winfo_children():
            widget.destroy()

        for i, photo in enumerate(self.photos):
            thumb = photo.resize((50, 50), Image.ANTIALIAS)
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
        plt.xlabel("Photo Index", fontsize=12)
        plt.ylabel("Greyscale Value (0-255)", fontsize=12)
        
        # Set x-ticks and y-ticks with labels
        plt.xticks(range(1, 13), [f"{i}" for i in range(1, 13)], rotation=45)
        plt.yticks(range(0, 256, 51), [str(i) for i in range(0, 256, 51)])
        
        plt.grid(True)
        plt.tight_layout()
        plt.show()




if __name__ == "__main__":
    root = tk.Tk()
    app = PhotoViewerApp(root)
    root.mainloop()
