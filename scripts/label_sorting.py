import os
import shutil

# ====================================
# Configurable Paths and Variables
# ====================================
ROOT_DIR = "."  # Directory where the script is located
LABELS_DIR = os.path.join(ROOT_DIR, "Labels")  # Folder containing label files
SUBSETS = ["test", "train", "val"]  # Dataset subsets
IMAGES_SUBDIR = "images"  # Folder name inside each subset where images are stored
LABELS_SUBDIR = "labels"  # Destination subfolder for copied label files
LABEL_EXTENSION = ".txt"  # Expected extension for label files

# Debug: Print the current working directory
print("Current working directory:", os.getcwd())

for subset in SUBSETS:
    subset_dir = os.path.join(ROOT_DIR, subset)
    images_dir = os.path.join(subset_dir, IMAGES_SUBDIR)
    dest_labels_dir = os.path.join(subset_dir, LABELS_SUBDIR)
    
    # Create the destination labels folder if it doesn't exist
    os.makedirs(dest_labels_dir, exist_ok=True)
    
    print(f"\nProcessing subset: {subset}")
    if not os.path.isdir(images_dir):
        print(f"Images directory '{images_dir}' does not exist. Skipping '{subset}'.")
        continue
    
    # List files in the images directory
    files = os.listdir(images_dir)
    print(f"Files in '{images_dir}': {files}")
    
    # Loop through each file in the images directory
    for filename in files:
        image_file_path = os.path.join(images_dir, filename)
        
        # Skip directories
        if os.path.isdir(image_file_path):
            continue
        
        # Extract base name to create label file name
        base_name, _ = os.path.splitext(filename)
        label_filename = base_name + LABEL_EXTENSION
        label_source_path = os.path.join(LABELS_DIR, label_filename)
        
        if os.path.isfile(label_source_path):
            # Copy the label file to the destination labels folder
            shutil.copy(label_source_path, os.path.join(dest_labels_dir, label_filename))
            print(f"Copied label '{label_filename}' to '{dest_labels_dir}'.")
        else:
            print(f"Label file '{label_source_path}' not found for image '{filename}'.")
