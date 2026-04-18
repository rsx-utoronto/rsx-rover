# Data augmentation script for object detection
# Balances classes by augmenting minority classes more heavily
# Uses Albumentations for robust augmentation with bounding box support

import os
import cv2
import numpy as np
import albumentations as A
from collections import defaultdict
import random
import time

# Split ratios of classes with null cases (should be ignored)
# Class 2: 2187 images -> train: 1749, val: 218, test: 220
# Class -1: 1324 images -> train: 1059, val: 132, test: 133 --> NULL
# Class 1: 5652 images -> train: 4521, val: 565, test: 566
# Class 0: 3023 images -> train: 2418, val: 302, test: 303
#
# Augmentation Strategy Options:
# 1. 'balance_only': Balance to majority class count (4,521 each) - conservative
# 2. 'balanced_growth': Balance to 6,000 each - RECOMMENDED for robust dataset
# 3. 'aggressive': Balance to 8,000 each - maximum data augmentation
# 4. Custom number: Set TARGET_COUNT to any number

train_path = r"C:\Users\Jakkii\OneDrive - University of Toronto\Documents\comp_data_sorted\train"

# Augmentation strategy
STRATEGY = 'balanced_growth'  # Options: 'balance_only', 'balanced_growth', 'aggressive', 'custom'
TARGET_COUNT = 6000  # Only used if STRATEGY = 'custom'

def get_augmentation_pipeline(strength='medium'):
    """
    Create augmentation pipeline for object detection.
    Handles bounding boxes properly for YOLO format.
    
    Args:
        strength: 'light', 'medium', or 'heavy' - determines augmentation intensity
    """
    if strength == 'light':
        transforms = [
            A.HorizontalFlip(p=0.5),
            A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
            A.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=20, val_shift_limit=10, p=0.3),
        ]
    elif strength == 'medium':
        transforms = [
            A.HorizontalFlip(p=0.5),
            A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.7),
            A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=30, val_shift_limit=20, p=0.5),
            A.OneOf([
                A.MotionBlur(blur_limit=5, p=1.0),
                A.GaussianBlur(blur_limit=5, p=1.0),
                A.MedianBlur(blur_limit=5, p=1.0),
            ], p=0.3),
            A.GaussNoise(p=0.3),
            A.RandomGamma(gamma_limit=(80, 120), p=0.3),
            A.CLAHE(clip_limit=4.0, p=0.3),
        ]
    else:  # heavy - simplified to avoid slow transforms
        transforms = [
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.2),
            A.RandomBrightnessContrast(brightness_limit=0.4, contrast_limit=0.4, p=0.8),
            A.HueSaturationValue(hue_shift_limit=25, sat_shift_limit=40, val_shift_limit=25, p=0.6),
            A.OneOf([
                A.MotionBlur(blur_limit=7, p=1.0),
                A.GaussianBlur(blur_limit=7, p=1.0),
                A.MedianBlur(blur_limit=5, p=1.0),
            ], p=0.5),
            A.GaussNoise(p=0.4),
            A.RandomGamma(gamma_limit=(70, 130), p=0.4),
            A.CLAHE(clip_limit=4.0, p=0.4),
            A.RandomRotate90(p=0.3),
            A.Rotate(limit=15, border_mode=cv2.BORDER_CONSTANT, p=0.4),
            A.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.3, hue=0.1, p=0.5),
            A.Sharpen(alpha=(0.2, 0.5), lightness=(0.5, 1.0), p=0.3),
            A.Blur(blur_limit=3, p=0.2),
        ]
    
    return A.Compose(
        transforms,
        bbox_params=A.BboxParams(
            format='yolo',
            label_fields=['class_labels'],
            min_visibility=0.3,
            min_area=100
        )
    )

def read_yolo_labels(label_path):
    """
    Read YOLO format labels from file.
    Returns list of [class_id, x_center, y_center, width, height]
    Validates and clamps bbox coordinates to [0, 1] range.
    """
    if not os.path.exists(label_path):
        return []
    
    labels = []
    with open(label_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                parts = line.split()
                class_id = int(parts[0])
                bbox = [float(x) for x in parts[1:5]]
                
                # Clamp bbox coordinates to valid YOLO range [0, 1]
                bbox[0] = max(0.0, min(1.0, bbox[0]))  # x_center
                bbox[1] = max(0.0, min(1.0, bbox[1]))  # y_center
                bbox[2] = max(0.0, min(1.0, bbox[2]))  # width
                bbox[3] = max(0.0, min(1.0, bbox[3]))  # height
                
                # Validate bbox has valid dimensions
                if bbox[2] > 0 and bbox[3] > 0:
                    labels.append([class_id] + bbox)
    return labels

def write_yolo_labels(label_path, labels):
    """
    Write YOLO format labels to file.
    """
    with open(label_path, 'w') as f:
        for label in labels:
            class_id = int(label[0])
            bbox = label[1:5]
            f.write(f"{class_id} {' '.join([f'{x:.6f}' for x in bbox])}\n")

def get_image_primary_class(label_path):
    """
    Get the primary class (minimum class ID) from a label file.
    Returns -1 if no valid labels or only null labels.
    """
    labels = read_yolo_labels(label_path)
    if not labels:
        return -1
    
    classes = [label[0] for label in labels]
    non_null_classes = [c for c in classes if c != -1]
    
    if not non_null_classes:
        return -1
    
    return min(non_null_classes)

def augment_image(image_path, label_path, output_image_path, output_label_path, pipeline):
    """
    Augment a single image and its labels.
    Returns True if successful, False otherwise.
    """
    try:
        # Read image
        image = cv2.imread(image_path)
        if image is None:
            return False
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Read labels (now with validation and clamping)
        labels = read_yolo_labels(label_path)
        if not labels:
            return False
        
        # Separate bboxes and class labels for albumentations
        bboxes = [label[1:5] for label in labels]
        class_labels = [label[0] for label in labels]
        
        # Validate all bboxes are in valid range after reading
        valid = True
        for bbox in bboxes:
            if not (0 <= bbox[0] <= 1 and 0 <= bbox[1] <= 1 and 
                    0 < bbox[2] <= 1 and 0 < bbox[3] <= 1):
                valid = False
                break
        
        if not valid:
            return False
        
        # Apply augmentation
        transformed = pipeline(image=image, bboxes=bboxes, class_labels=class_labels)
        
        # Check if any bboxes remain after augmentation
        if not transformed['bboxes']:
            return False
        
        # Save augmented image
        aug_image = cv2.cvtColor(transformed['image'], cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_image_path, aug_image)
        
        # Save augmented labels
        aug_labels = [[transformed['class_labels'][i]] + list(bbox) 
                      for i, bbox in enumerate(transformed['bboxes'])]
        write_yolo_labels(output_label_path, aug_labels)
        
        return True
    
    except KeyboardInterrupt:
        raise  # Allow user to stop the process
    except Exception as e:
        # Silently fail for individual images to avoid cluttering output
        return False

def balance_dataset(train_path, strategy='balanced_growth', custom_target=6000):
    """
    Balance the dataset by augmenting minority classes.
    
    Args:
        train_path: Path to training data
        strategy: 'balance_only', 'balanced_growth', 'aggressive', or 'custom'
        custom_target: Target count if strategy is 'custom'
    """
    images_dir = os.path.join(train_path, 'images')
    labels_dir = os.path.join(train_path, 'labels')
    
    print("Scanning dataset...")
    print("Note: Invalid bounding boxes will be automatically fixed")
    
    # Group images by class
    class_images = defaultdict(list)
    image_files = [f for f in os.listdir(images_dir) 
                   if f.endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    
    for img_file in image_files:
        base_name = os.path.splitext(img_file)[0]
        label_path = os.path.join(labels_dir, base_name + '.txt')
        
        primary_class = get_image_primary_class(label_path)
        class_images[primary_class].append(img_file)
    
    # Print current distribution
    print("\nCurrent class distribution:")
    for class_id in sorted(class_images.keys()):
        if class_id != -1:
            print(f"  Class {class_id}: {len(class_images[class_id])} images")
    
    # Determine target count based on strategy
    current_max = max(len(class_images[c]) for c in class_images.keys() if c != -1)
    
    if strategy == 'balance_only':
        target_count = current_max
    elif strategy == 'balanced_growth':
        target_count = 6000
    elif strategy == 'aggressive':
        target_count = 8000
    elif strategy == 'custom':
        target_count = custom_target
    else:
        print(f"Unknown strategy '{strategy}', defaulting to balanced_growth")
        target_count = 6000
    
    print(f"\nStrategy: {strategy}")
    print(f"Target count per class: {target_count}")
    print(f"This will create a total of ~{target_count * 3} training images")
    
    # Show augmentation plan
    print("\nAugmentation Plan:")
    print("-" * 60)
    for class_id in sorted(class_images.keys()):
        if class_id == -1:
            continue
        current = len(class_images[class_id])
        needed = max(0, target_count - current)
        if needed > 0:
            aug_ratio = needed / current
            print(f"  Class {class_id}: {current:>5} → {target_count:>5} (+{needed:>5} augmented, {aug_ratio:.2f}x)")
        else:
            print(f"  Class {class_id}: {current:>5} (no augmentation needed)")
    print("-" * 60)
    
    # Augment each class to reach target
    for class_id in sorted(class_images.keys()):
        if class_id == -1:
            print(f"\nSkipping class {class_id} (null class)")
            continue
        
        current_count = len(class_images[class_id])
        needed = target_count - current_count
        
        if needed <= 0:
            print(f"\nClass {class_id}: Already at or above target ({current_count} images, target: {target_count})")
            continue
        
        print(f"\nClass {class_id}: Augmenting {current_count} → {target_count} (generating {needed} more images)...")
        
        # Determine augmentation strength based on how many we need
        augmentations_per_image = needed / current_count
        if augmentations_per_image < 0.5:
            strength = 'light'
        elif augmentations_per_image < 1.5:
            strength = 'medium'
        else:
            strength = 'heavy'
        
        print(f"  Using '{strength}' augmentation (need {augmentations_per_image:.2f}x augmentations)")
        
        # Create augmentation pipeline
        pipeline = get_augmentation_pipeline(strength)
        
        # Find the highest existing augmentation number to continue from
        print(f"  Checking for existing augmented images...")
        max_aug_number = -1
        for existing_file in os.listdir(images_dir):
            if '_aug_' in existing_file:
                try:
                    # Extract the augmentation number from filename
                    aug_part = existing_file.split('_aug_')[1]
                    aug_num = int(aug_part.split('.')[0])
                    max_aug_number = max(max_aug_number, aug_num)
                except:
                    pass
        
        start_aug_number = max_aug_number + 1
        if start_aug_number > 0:
            print(f"  Found existing augmentations up to _aug_{max_aug_number}, starting from _aug_{start_aug_number}")
        
        # Augment images
        images_to_augment = class_images[class_id].copy()
        augmented_count = 0
        attempts = 0
        max_attempts = needed * 5  # Allow some failures
        aug_counter = start_aug_number  # Start from next available number
        start_time = time.time()
        
        print(f"  Starting augmentation at {time.strftime('%H:%M:%S')}")
        
        while augmented_count < needed and attempts < max_attempts:
            # Randomly select an image to augment
            img_file = random.choice(images_to_augment)
            base_name = os.path.splitext(img_file)[0]
            
            # Create unique name for augmented image with global counter
            aug_suffix = f"_aug_{aug_counter}"
            aug_base_name = base_name + aug_suffix
            aug_img_file = aug_base_name + os.path.splitext(img_file)[1]
            
            # Paths
            src_image_path = os.path.join(images_dir, img_file)
            src_label_path = os.path.join(labels_dir, base_name + '.txt')
            dst_image_path = os.path.join(images_dir, aug_img_file)
            dst_label_path = os.path.join(labels_dir, aug_base_name + '.txt')
            
            # Check if file already exists (safety check)
            if os.path.exists(dst_image_path):
                aug_counter += 1
                attempts += 1
                continue
            
            # Augment
            if augment_image(src_image_path, src_label_path, dst_image_path, dst_label_path, pipeline):
                augmented_count += 1
                aug_counter += 1
                if augmented_count % 50 == 0:  # More frequent updates
                    elapsed = time.time() - start_time
                    rate = augmented_count / elapsed if elapsed > 0 else 0
                    remaining = (needed - augmented_count) / rate if rate > 0 else 0
                    print(f"  Progress: {augmented_count}/{needed} ({augmented_count*100//needed}%) | "
                          f"{rate:.1f} img/s | ETA: {remaining/60:.1f} min")
            else:
                aug_counter += 1  # Increment even on failure to avoid filename collisions
            
            attempts += 1
            
            # Show periodic heartbeat even without successful augmentations
            if attempts % 200 == 0 and augmented_count % 50 != 0:
                elapsed = time.time() - start_time
                print(f"  Working... {augmented_count}/{needed} completed in {elapsed/60:.1f} min ({attempts} attempts)")
        
        elapsed_total = time.time() - start_time
        if augmented_count < needed:
            print(f"  Warning: Only generated {augmented_count}/{needed} augmented images (some augmentations failed)")
        else:
            print(f"  Completed: Generated {augmented_count} augmented images")
        print(f"  New total for class {class_id}: {current_count + augmented_count}")
        print(f"  Time taken: {elapsed_total/60:.1f} minutes ({elapsed_total/augmented_count:.2f} sec/image)" if augmented_count > 0 else "")
    
    print("\n" + "="*60)
    print("Dataset balancing complete!")
    print("="*60)
    
    # Final count
    print("\nFinal class distribution:")
    class_images_final = defaultdict(list)
    image_files_final = [f for f in os.listdir(images_dir) 
                         if f.endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    
    for img_file in image_files_final:
        base_name = os.path.splitext(img_file)[0]
        label_path = os.path.join(labels_dir, base_name + '.txt')
        primary_class = get_image_primary_class(label_path)
        class_images_final[primary_class].append(img_file)
    
    for class_id in sorted(class_images_final.keys()):
        if class_id != -1:
            print(f"  Class {class_id}: {len(class_images_final[class_id])} images")

if __name__ == "__main__":
    print("="*60)
    print("DATA AUGMENTATION FOR OBJECT DETECTION")
    print("="*60)
    print(f"Train path: {train_path}")
    print(f"Strategy: {STRATEGY}")
    print("\nThis will augment images to balance classes 0, 1, and 2")
    print("Class -1 (null) will be skipped")
    print("Augmented images will be saved with '_aug_N' suffix")
    print("-" * 60)
    
    balance_dataset(train_path, strategy=STRATEGY, custom_target=TARGET_COUNT)
