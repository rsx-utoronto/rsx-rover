# Data split script for object detection
# 80 10 10 split train, val, test with stratified sampling

import os
import shutil
import random
from collections import defaultdict

# Path to the dataset (unsplit with images and labels)
# Local to Jakkii's computer
dataset_path = r"C:\Users\Jakkii\OneDrive - University of Toronto\Documents\comp_data"
output_path = r"C:\Users\Jakkii\OneDrive - University of Toronto\Documents\comp_data_sorted"

def get_image_classes(image_file, labels_path):
    """
    Get the classes present in an image by reading its label file.
    Assumes YOLO format where label files have same name as image but .txt extension.
    Returns a set of class IDs found in the label file.
    """
    # Get the label file name (replace image extension with .txt)
    base_name = os.path.splitext(image_file)[0]
    label_file = os.path.join(labels_path, base_name + '.txt')
    
    classes = set()
    if os.path.exists(label_file):
        with open(label_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    # First value in each line is the class ID
                    class_id = int(line.split()[0])
                    classes.add(class_id)
    return classes

def stratified_split(image_files, labels_path, split_ratio=(0.8, 0.1, 0.1)):
    """
    Perform stratified split to maintain class distribution across splits.
    Groups images by their primary class and splits each group proportionally.
    """
    # Group images by their classes (use primary class for simplicity)
    class_groups = defaultdict(list)
    
    for img_file in image_files:
        classes = get_image_classes(img_file, labels_path)
        if classes:
            # Use the minimum class ID as primary class for grouping
            primary_class = min(classes)
            class_groups[primary_class].append(img_file)
        else:
            # Images without labels go to a special group
            class_groups[-1].append(img_file)
    
    # Shuffle each class group
    for class_id in class_groups:
        random.shuffle(class_groups[class_id])
    
    # Split each class group proportionally
    train_files, val_files, test_files = [], [], []
    
    for class_id, files in class_groups.items():
        num_files = len(files)
        num_train = int(num_files * split_ratio[0])
        num_val = int(num_files * split_ratio[1])
        
        train_files.extend(files[:num_train])
        val_files.extend(files[num_train:num_train + num_val])
        test_files.extend(files[num_train + num_val:])
        
        print(f"Class {class_id}: {num_files} images -> train: {len(files[:num_train])}, "
              f"val: {len(files[num_train:num_train + num_val])}, "
              f"test: {len(files[num_train + num_val:])}")
    
    return train_files, val_files, test_files

def copy_files(image_files, images_dir, labels_dir, dst_dir):
    """
    Copy image files and their corresponding label files to destination directory.
    Creates images/ and labels/ subdirectories in the destination.
    """
    # Create images and labels subdirectories in destination
    dst_images_dir = os.path.join(dst_dir, 'images')
    dst_labels_dir = os.path.join(dst_dir, 'labels')
    os.makedirs(dst_images_dir, exist_ok=True)
    os.makedirs(dst_labels_dir, exist_ok=True)
    
    for img_file in image_files:
        # Copy image file
        src_img = os.path.join(images_dir, img_file)
        dst_img = os.path.join(dst_images_dir, img_file)
        shutil.copy(src_img, dst_img)
        
        # Copy corresponding label file if it exists
        base_name = os.path.splitext(img_file)[0]
        label_file = base_name + '.txt'
        src_label = os.path.join(labels_dir, label_file)
        dst_label = os.path.join(dst_labels_dir, label_file)
        
        if os.path.exists(src_label):
            shutil.copy(src_label, dst_label)

def data_split(dataset_path, output_path, split_ratio=(0.8, 0.1, 0.1)):
    """
    Split the dataset into train, val, test sets with stratified sampling.
    Maintains equal class distribution across all splits.
    Expects dataset_path to contain 'images/' and 'labels/' subdirectories.
    """
    print(f"Reading dataset from: {dataset_path}")
    print(f"Output will be saved to: {output_path}")
    
    # Define paths to images and labels subdirectories
    images_path = os.path.join(dataset_path, 'images')
    labels_path = os.path.join(dataset_path, 'labels')
    
    # Check if subdirectories exist
    if not os.path.exists(images_path):
        print(f"Error: images directory not found at {images_path}")
        return
    if not os.path.exists(labels_path):
        print(f"Error: labels directory not found at {labels_path}")
        return
    
    # Get all image files in the images subdirectory
    image_files = [f for f in os.listdir(images_path) 
                   if f.endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff'))]
    
    print(f"\nFound {len(image_files)} images")
    
    if len(image_files) == 0:
        print("No images found! Please check the dataset path.")
        return
    
    # Perform stratified split
    print("\nPerforming stratified split:")
    train_files, val_files, test_files = stratified_split(image_files, labels_path, split_ratio)
    
    # Create output directories
    train_dir = os.path.join(output_path, 'train')
    val_dir = os.path.join(output_path, 'val')
    test_dir = os.path.join(output_path, 'test')
    
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)
    
    # Copy files to their respective directories
    print(f"\nCopying files to {output_path}...")
    print(f"Copying {len(train_files)} files to train/")
    copy_files(train_files, images_path, labels_path, train_dir)
    
    print(f"Copying {len(val_files)} files to val/")
    copy_files(val_files, images_path, labels_path, val_dir)
    
    print(f"Copying {len(test_files)} files to test/")
    copy_files(test_files, images_path, labels_path, test_dir)
    
    print(f"\n✓ Split complete!")
    print(f"Total: {len(image_files)} images -> train: {len(train_files)}, "
          f"val: {len(val_files)}, test: {len(test_files)}")

if __name__ == "__main__":
    data_split(dataset_path, output_path)