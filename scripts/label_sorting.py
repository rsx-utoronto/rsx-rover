#!/usr/bin/python3

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

# ====================================
# Process each subset directory
# ====================================
for subset in SUBSETS:
    subset_dir = os.path.join(ROOT_DIR, subset)
    # Create the destination labels folder in each subset folder
    dest_labels_dir = os.path.join(subset_dir, LABELS_SUBDIR)
    os.makedirs(dest_labels_dir, exist_ok=True)
    
    # Check if the subset directory exists
    if not os.path.isdir(subset_dir):
        print(f"Directory '{subset_dir}' does not exist. Skipping '{subset}'.")
        continue

    # Loop through each file in the subset directory (ignoring the labels subfolder)
    for filename in os.listdir(subset_dir):
        file_path = os.path.join(subset_dir, filename)
        # Skip directories (like the 'labels' folder we just created)
        if os.path.isdir(file_path):
            continue
        
        # Extract the base name to construct the label file name
        base_name, _ = os.path.splitext(filename)
        label_file_name = base_name + LABEL_EXTENSION
        label_source_path = os.path.join(LABELS_DIR, label_file_name)
        
        # Check if the corresponding label file exists
        if os.path.isfile(label_source_path):
            # Copy the label file to the subset's labels folder
            shutil.copy(label_source_path, os.path.join(dest_labels_dir, label_file_name))
            print(f"Copied label '{label_file_name}' to '{dest_labels_dir}'.")
        else:
            print(f"Label file '{label_source_path}' not found for image '{filename}'.")
