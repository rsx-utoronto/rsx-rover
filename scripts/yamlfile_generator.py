import os
import shutil
import random

# Paths to your images and labels directories
image_dir = '/home/rsx-base/comp_data/Images'
label_dir = '/home/rsx-base/comp_data/Labels'

# Paths to the train/val/test directories
train_image_dir = 'images/train'
train_label_dir = 'labels/train'
val_image_dir = 'images/val'
val_label_dir = 'labels/val'
test_image_dir = 'images/test'
test_label_dir = 'labels/test'

# Create directories if they don’t exist
for dir_path in [train_image_dir, train_label_dir, val_image_dir, val_label_dir, test_image_dir, test_label_dir]:
    try:
        if os.path.exists(dir_path):
            # Remove the directory and all its contents
            shutil.rmtree(dir_path)
            print(f"Deleted existing directory: {dir_path}")
        
        # Create the directory
        os.makedirs(dir_path)
        print(f"Created directory: {dir_path}")
    
    except Exception as e:
        print(f"Error handling directory {dir_path}: {e}")

# Get a list of all images
images = [f for f in os.listdir(image_dir)] #if f.lower().endswith('.jpg')]

# Get a list of all labels
labels = [f for f in os.listdir(label_dir)] #if f.lower().endswith('.txt')]
print(images)
print()

# Shuffle images randomly
random.shuffle(images)

print(images)
# Calculate split sizes
total_images = len(images)
print(total_images)
train_size = int(total_images * 0.6)
val_size = int(total_images * 0.3)

print("train size",train_size)
print(val_size)

# Split the images
train_images = images[:train_size]
val_images = images[train_size:train_size + val_size]
test_images = images[train_size + val_size:]



def move_files(file_list, dest_img_dir, dest_lbl_dir):
    for img_name in file_list:
        label_name = img_name.replace('.jpg', '.txt')
        if label_name in labels:
            # Move image
            shutil.copy(os.path.join(image_dir, img_name), os.path.join(dest_img_dir, img_name))
            # Move label
            shutil.copy(os.path.join(label_dir, label_name), os.path.join(dest_lbl_dir, label_name))

# Move files to the respective directories
move_files(train_images, train_image_dir, train_label_dir)
move_files(val_images, val_image_dir, val_label_dir)
move_files(test_images, test_image_dir, test_label_dir)

print("Data split complete!")

# Step 2: Create the data.yaml file
yaml_content = f"""
train: {train_image_dir}
val: {val_image_dir}
test: {test_image_dir}

# Number of classes
nc: 2  # Adjust based on your dataset

# Class names
names: 
    0: waterbottle
    1: mallet
# Replace with your class names
"""

# Write the YAML content to a file
with open('datafinal.yaml', 'w') as yaml_file:
    yaml_file.write(yaml_content.strip())

print("data.yaml file created successfully!")