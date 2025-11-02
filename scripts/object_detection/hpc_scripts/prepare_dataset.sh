#!/bin/bash

# ============================================================================
# Dataset Preparation Script for HPC
# This script creates a tar.gz archive of your dataset for HPC upload
# ============================================================================

echo "============================================================"
echo "Dataset Preparation for HPC Training"
echo "============================================================"
echo ""

# Check if comp_data_sorted exists
if [ ! -d "comp_data_sorted" ]; then
    echo "❌ Error: comp_data_sorted directory not found!"
    echo "Please run this script from the directory containing comp_data_sorted/"
    echo ""
    echo "Expected structure:"
    echo "  comp_data_sorted/"
    echo "    ├── train/"
    echo "    │   ├── images/"
    echo "    │   └── labels/"
    echo "    ├── val/"
    echo "    │   ├── images/"
    echo "    │   └── labels/"
    echo "    └── test/"
    echo "        ├── images/"
    echo "        └── labels/"
    exit 1
fi

# Verify structure
echo "Checking dataset structure..."
for split in train val test; do
    if [ ! -d "comp_data_sorted/$split/images" ] || [ ! -d "comp_data_sorted/$split/labels" ]; then
        echo "❌ Error: Missing comp_data_sorted/$split/images or labels directory!"
        exit 1
    fi
    
    img_count=$(ls comp_data_sorted/$split/images/ | wc -l)
    lbl_count=$(ls comp_data_sorted/$split/labels/ | wc -l)
    echo "  ✅ $split: $img_count images, $lbl_count labels"
done

echo ""
echo "Dataset structure verified!"
echo ""

# Check if tar.gz already exists
if [ -f "comp_data_sorted.tar.gz" ]; then
    echo "⚠️  Warning: comp_data_sorted.tar.gz already exists!"
    read -p "Do you want to overwrite it? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi
    rm -f comp_data_sorted.tar.gz
fi

# Create tar.gz
echo "Creating tar.gz archive..."
echo "This may take a few minutes for large datasets..."
echo ""

tar -czf comp_data_sorted.tar.gz comp_data_sorted/ --checkpoint=1000 --checkpoint-action=echo="Processed %{checkpoint}k files"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Archive created successfully!"
    echo ""
    
    # Display file size
    size=$(du -h comp_data_sorted.tar.gz | cut -f1)
    echo "File: comp_data_sorted.tar.gz"
    echo "Size: $size"
    echo ""
    
    # Count total files in archive
    total_files=$(tar -tzf comp_data_sorted.tar.gz | wc -l)
    echo "Total files in archive: $total_files"
    echo ""
    
    # Display upload instructions
    echo "============================================================"
    echo "Next Steps:"
    echo "============================================================"
    echo ""
    echo "1. Convert to .tar (uncompressed for faster extraction):"
    echo "   tar -xzf comp_data_sorted.tar.gz"
    echo "   tar -cf comp_data_sorted.tar comp_data_sorted/"
    echo ""
    echo "2. Upload to HPC:"
    echo "   scp comp_data_sorted.tar killarney:~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/"
    echo ""
    echo "3. Upload training scripts (if not already there):"
    echo "   scp -r scripts/object_detection/hpc_scripts killarney:~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/scripts/object_detection/"
    echo ""
    echo "4. SSH to HPC and submit job:"
    echo "   ssh killarney"
    echo "   cd ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection"
    echo "   sbatch scripts/object_detection/hpc_scripts/run_training.sh"
    echo ""
    echo "See README.md for detailed instructions."
    echo "============================================================"
    
else
    echo "❌ Error: Failed to create archive!"
    exit 1
fi

