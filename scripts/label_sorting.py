import os
import shutil

# =======================
# Configurable Variables
# =======================
BASE_DIR = "updated_data"       # Folder containing test, train, val
LABELS_DIR = "Labels"           # Folder containing label files
IMAGE_FOLDER_NAME = "images"    # Folder name inside each subset that contains images
OUTPUT_LABELS_SUBDIR = "labels" # Subfolder where label files will be copied
LABEL_EXTENSION = ".txt"        # Extension for label files (adjust if needed)

# =======================
# Process each dataset subset
# =======================
for subset in ["test", "train", "val"]:
    # Define the path to the images and the destination for labels
    images_path = os.path.join(BASE_DIR, subset, IMAGE_FOLDER_NAME)
    output_labels_path = os.path.join(BASE_DIR, subset, OUTPUT_LABELS_SUBDIR)
    
    # Create the output labels folder if it doesn't exist
    os.makedirs(output_labels_path, exist_ok=True)
    
    # Check if the images folder exists
    if not os.path.isdir(images_path):
        print(f"Warning: Images folder '{images_path}' does not exist. Skipping '{subset}'.")
        continue
    
    # Loop through each image in the images folder
    for image_file in os.listdir(images_path):
        image_file_path = os.path.join(images_path, image_file)
        if os.path.isfile(image_file_path):
            # Get the file name without extension (e.g., "image1" from "image1.jpg")
            base_name, _ = os.path.splitext(image_file)
            
            # Construct the expected label file name and path
            label_file_name = base_name + LABEL_EXTENSION
            label_source_path = os.path.join(LABELS_DIR, label_file_name)
            
            if os.path.isfile(label_source_path):
                # Copy the label file to the output labels folder
                shutil.copy(label_source_path, os.path.join(output_labels_path, label_file_name))
                print(f"Copied label for '{image_file}' to '{output_labels_path}'.")
            else:
                print(f"Label file '{label_source_path}' not found for image '{image_file}'.")
