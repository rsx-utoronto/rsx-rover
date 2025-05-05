#!/usr/bin/python3

import os
import shutil

# ====================================
# Configurable Paths and Variables
# ====================================
# ROOT_DIR: the directory where the script is located (adjust if needed)
ROOT_DIR = "."
# Folder containing label files (assumed to be in the same root)
LABELS_DIR = os.path.join(ROOT_DIR, "Labels")
# List of dataset subsets (directories in the same root as the script)
SUBSETS = ["test", "train", "val"]
# Name of the subfolder inside each subset where labels will be copied
LABELS_SUBDIR = "labels"
# Expected file extension for label files (adjust as needed)
LABEL_EXTENSION = ".txt"

# ====================================
# Process each subset directory
# ====================================
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
