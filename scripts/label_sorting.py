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
