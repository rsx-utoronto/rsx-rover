import os
import shutil
import random

# Paths to your images and labels directories
image_dir = 'dataset/images'
label_dir = 'dataset/labels'

# Paths to the train/val/test directories
train_image_dir = 'train/images'
train_label_dir = 'train/labels'
val_image_dir = 'val/images'
val_label_dir = 'val/labels'
test_image_dir = 'test/images'
test_label_dir = 'test/labels'

# Create directories if they don’t exist
for dir_path in [train_image_dir, train_label_dir, val_image_dir, val_label_dir, test_image_dir, test_label_dir]:
    os.makedirs(dir_path, exist_ok=True)

# Get a list of all images
images = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]

# Shuffle images randomly
random.shuffle(images)

# Calculate split sizes
total_images = len(images)
train_size = int(total_images * 0.6)
val_size = int(total_images * 0.3)

# Split the images
train_images = images[:train_size]
val_images = images[train_size:train_size + val_size]
test_images = images[train_size + val_size:]

def move_files(file_list, dest_img_dir, dest_lbl_dir):
    for img_name in file_list:
        label_name = img_name.replace('.jpg', '.txt')
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
nc: 1  # Adjust based on your dataset

# Class names
names: ['plastic_bottle']  # Replace with your class names
"""

# Write the YAML content to a file
with open('data.yaml', 'w') as yaml_file:
    yaml_file.write(yaml_content.strip())

print("data.yaml file created successfully!")