"""
Script to fix YOLO format labels by changing all class IDs to 2.
Handles empty/null label files gracefully.

YOLO format: class_id x_center y_center width height
"""

import os
import sys

def fix_label_file(file_path, target_class=2):
    """
    Fix a single YOLO label file by changing all class IDs to target_class.
    
    Args:
        file_path: Path to the label file
        target_class: The class ID to change all labels to (default: 2)
    
    Returns:
        tuple: (success, num_lines_changed)
    """
    try:
        # Read the file
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # Handle empty files
        if not lines or all(line.strip() == '' for line in lines):
            print(f"  Skipping empty file: {file_path}")
            return True, 0
        
        # Process each line
        modified_lines = []
        lines_changed = 0
        
        for line in lines:
            line = line.strip()
            if not line:  # Skip empty lines
                continue
                
            parts = line.split()
            if len(parts) >= 5:  # Valid YOLO format: class x y w h
                old_class = parts[0]
                if old_class != str(target_class):
                    parts[0] = str(target_class)
                    lines_changed += 1
                modified_lines.append(' '.join(parts) + '\n')
            else:
                print(f"  Warning: Invalid format in {file_path}: {line}")
                modified_lines.append(line + '\n')
        
        # Write back to file
        with open(file_path, 'w') as f:
            f.writelines(modified_lines)
        
        return True, lines_changed
    
    except Exception as e:
        print(f"  Error processing {file_path}: {e}")
        return False, 0

def fix_labels_in_directory(directory_path, target_class=2):
    """
    Fix all YOLO label files in a directory.
    
    Args:
        directory_path: Path to directory containing .txt label files
        target_class: The class ID to change all labels to (default: 2)
    """
    if not os.path.isdir(directory_path):
        print(f"Error: {directory_path} is not a valid directory")
        return
    
    # Find all .txt files
    label_files = [f for f in os.listdir(directory_path) if f.endswith('.txt')]
    
    if not label_files:
        print(f"No .txt files found in {directory_path}")
        return
    
    print(f"Found {len(label_files)} label files in {directory_path}")
    print(f"Changing all class IDs to {target_class}...\n")
    
    total_files = 0
    total_lines_changed = 0
    failed_files = 0
    
    for label_file in label_files:
        file_path = os.path.join(directory_path, label_file)
        success, lines_changed = fix_label_file(file_path, target_class)
        
        if success:
            total_files += 1
            total_lines_changed += lines_changed
            if lines_changed > 0:
                print(f"✓ {label_file}: Changed {lines_changed} label(s)")
        else:
            failed_files += 1
    
    print(f"\n{'='*60}")
    print(f"Summary:")
    print(f"  Total files processed: {total_files}")
    print(f"  Total labels changed: {total_lines_changed}")
    print(f"  Failed files: {failed_files}")
    print(f"{'='*60}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python fix_labelling.py <directory_path> [target_class]")
        print("  directory_path: Path to directory containing YOLO label files")
        print("  target_class: Class ID to change all labels to (default: 2)")
        print("\nExample: python fix_labelling.py ./labels 2")
        sys.exit(1)
    
    directory_path = sys.argv[1]
    target_class = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    
    fix_labels_in_directory(directory_path, target_class)

if __name__ == '__main__':
    main()

