import os
import shutil

# =======================
# Configurable Variables
# =======================
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
LABELS_DIR = os.path.join(ROOT_DIR, "Labels")
SUBSETS = ["test", "train", "val"]
IMAGES_SUBDIR = "images"
LABELS_SUBDIR = "labels"
LABEL_EXTENSION = ".txt"
# ====================================
# Process each subset directory
# ====================================
for subset in SUBSETS:
    subset_dir = os.path.join(ROOT_DIR, subset)
    images_dir = os.path.join(subset_dir, IMAGES_SUBDIR)
    # Create the destination labels folder in each subset folder
    dest_labels_dir = os.path.join(subset_dir, LABELS_SUBDIR)
    os.makedirs(dest_labels_dir, exist_ok=True)
    
    # Check if the subset directory exists
    if not os.path.isdir(images_dir):
        print(f"Directory '{subset_dir}' does not exist. Skipping '{subset}'.")
        continue

    files = os.listdir(images_dir)

    # Loop through each file in the subset directory (ignoring the labels subfolder)
    for filename in files:
        image_file_path = os.path.join(images_dir, filename)

        if os.path.isdir(image_file_path):
            continue

        base_name, _ = os.path.splitext(filename)
        label_filename = base_name + LABEL_EXTENSION
        label_source_path = os.path.join(LABELS_DIR, label_filename)

        if os.path.isfile(label_source_path):
            shutil.copy(label_source_path, os.path.join(dest_labels_dir, label_filename))
