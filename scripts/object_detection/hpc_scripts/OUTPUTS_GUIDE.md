# YOLO Training Outputs Guide

## Overview

After training completes, all outputs are saved to:
```
~/yolo_project/runs/detect/yolo12l_hpc_run_TIMESTAMP/
```

This guide explains all files that will be generated and how to use them.

---

## 📁 Output Directory Structure

```
yolo12l_hpc_run_TIMESTAMP/
├── weights/                      # Trained model weights
│   ├── best.pt                   # Best model (highest mAP)
│   ├── last.pt                   # Last epoch model
│   └── epoch*.pt                 # Checkpoints (every 10 epochs)
├── results.csv                   # Training metrics per epoch
├── results.png                   # Training curves plot
├── confusion_matrix.png          # Confusion matrix
├── confusion_matrix_normalized.png
├── F1_curve.png                  # F1 score curve
├── P_curve.png                   # Precision curve
├── R_curve.png                   # Recall curve
├── PR_curve.png                  # Precision-Recall curve
├── labels.jpg                    # Label distribution
├── labels_correlogram.jpg        # Label correlation
├── train_batch*.jpg              # Training batch examples
├── val_batch*.jpg                # Validation batch examples
├── args.yaml                     # Training configuration backup
└── predictions.json              # Validation predictions (if saved)
```

---

## 🔑 Key Files Explained

### 1. Model Weights (`weights/`)

#### `best.pt` ⭐ **USE THIS FOR INFERENCE**
- Saved when validation mAP improves
- Best performing model during training
- **This is the model you should use for deployment**

**Usage:**
```python
from ultralytics import YOLO
model = YOLO('path/to/best.pt')
results = model.predict('image.jpg')
```

#### `last.pt`
- Model from the last training epoch
- Useful if you want to resume training
- May not be the best performing model

#### `epoch*.pt`
- Checkpoint files saved every 10 epochs
- Useful for analyzing training progression
- Can resume from any checkpoint

**Resume training:**
```python
model = YOLO('path/to/last.pt')
model.train(data='data.yaml', resume=True)
```

---

### 2. Training Metrics (`results.csv`)

Contains per-epoch metrics in CSV format:

| Column | Description |
|--------|-------------|
| epoch | Epoch number |
| train/box_loss | Training bounding box loss |
| train/cls_loss | Training classification loss |
| train/dfl_loss | Training distribution focal loss |
| metrics/precision(B) | Precision (box) |
| metrics/recall(B) | Recall (box) |
| metrics/mAP50(B) | Mean Average Precision @ IoU 0.5 |
| metrics/mAP50-95(B) | Mean Average Precision @ IoU 0.5:0.95 |
| val/box_loss | Validation bounding box loss |
| val/cls_loss | Validation classification loss |
| val/dfl_loss | Validation distribution focal loss |
| lr/pg0, lr/pg1, lr/pg2 | Learning rates for parameter groups |

**Load and analyze:**
```python
import pandas as pd
df = pd.read_csv('results.csv')
print(df[['epoch', 'metrics/mAP50(B)', 'metrics/mAP50-95(B)']].tail())
```

---

### 3. Visualization Plots

#### `results.png` 📊
Combined plot showing:
- Training and validation losses
- Precision, Recall, mAP curves
- Learning rate schedule

**Best for:** Quick overview of training progress

#### `confusion_matrix.png` / `confusion_matrix_normalized.png`
Shows how well the model distinguishes between classes:
- Rows: True labels
- Columns: Predicted labels
- Diagonal: Correct predictions

**Interpreting:**
- High diagonal values = good performance
- Off-diagonal values = confusion between classes
- Normalized version shows percentages

#### `F1_curve.png`, `P_curve.png`, `R_curve.png`
- Show metrics at different confidence thresholds
- Helps choose optimal confidence threshold
- F1 = harmonic mean of Precision and Recall

#### `PR_curve.png` (Precision-Recall Curve)
- X-axis: Recall
- Y-axis: Precision
- Area Under Curve (AUC) indicates performance
- Higher AUC = better model

#### `labels.jpg`
Distribution of:
- Class frequencies
- Bounding box sizes
- Box center positions

**Use to:** Verify data balance and distribution

#### `labels_correlogram.jpg`
Shows correlation between:
- Box dimensions (width vs height)
- Box positions
- Class relationships

---

### 4. Batch Examples

#### `train_batch*.jpg` & `val_batch*.jpg`
Sample images with:
- Ground truth boxes (labels)
- Model predictions (if validation batch)
- Useful for visual inspection

**Check these to:**
- Verify data augmentation is working
- Ensure labels are correct
- See what the model is learning

---

### 5. Configuration Backup

#### `args.yaml`
Complete record of all training parameters:
- Model architecture
- Hyperparameters
- Data paths
- Augmentation settings

**Use to:** Reproduce exact training setup later

---

## 📊 Performance Metrics Explained

### mAP50 (Mean Average Precision @ IoU 0.5)
- **Good:** > 0.70
- **Excellent:** > 0.85
- Measures detection accuracy at 50% overlap threshold

### mAP50-95 (Mean Average Precision @ IoU 0.5:0.95)
- **Good:** > 0.50
- **Excellent:** > 0.70
- Stricter metric, averages over IoU 0.5 to 0.95
- **This is the standard metric for comparing models**

### Precision
- Percentage of correct detections out of all detections
- High precision = few false positives

### Recall
- Percentage of ground truth objects detected
- High recall = few false negatives (misses)

### Per-Class Performance
Your training outputs per-class metrics for:
- **Class 0:** waterbottle
- **Class 1:** mallet  
- **Class 2:** hammer

Check if any class is underperforming!

---

## 💡 Using Your Trained Model

### Basic Inference
```python
from ultralytics import YOLO

# Load model
model = YOLO('runs/detect/yolo12l_hpc_run_TIMESTAMP/weights/best.pt')

# Predict on single image
results = model.predict('image.jpg', save=True, conf=0.25)

# Predict on folder
results = model.predict('images/', save=True, conf=0.25)

# Get predictions
for result in results:
    boxes = result.boxes  # Bounding boxes
    print(f"Found {len(boxes)} objects")
    
    for box in boxes:
        cls = int(box.cls[0])  # Class ID
        conf = float(box.conf[0])  # Confidence
        xyxy = box.xyxy[0].tolist()  # Box coordinates
        print(f"Class: {cls}, Confidence: {conf:.2f}, Box: {xyxy}")
```

### Batch Processing
```python
import glob

# Process all images in a folder
image_paths = glob.glob('path/to/images/*.jpg')

for img_path in image_paths:
    results = model.predict(img_path, save=True)
    print(f"Processed: {img_path}")
```

### Export Model
```python
# Export to ONNX (for deployment)
model.export(format='onnx')

# Export to TensorRT (for NVIDIA GPUs)
model.export(format='engine')

# Export to CoreML (for Apple devices)
model.export(format='coreml')
```

### Validation
```python
# Validate on test set
metrics = model.val(data='path/to/data.yaml', split='test')

print(f"mAP50: {metrics.box.map50:.4f}")
print(f"mAP50-95: {metrics.box.map:.4f}")
print(f"Precision: {metrics.box.mp:.4f}")
print(f"Recall: {metrics.box.mr:.4f}")
```

---

## 🔍 Troubleshooting Poor Results

### If mAP is low (< 0.50):

1. **Check training curves** (`results.png`)
   - Are losses decreasing?
   - Is there overfitting (val loss increasing)?

2. **Check confusion matrix**
   - Which classes are confused?
   - Is one class performing poorly?

3. **Inspect batch examples**
   - Are augmentations too aggressive?
   - Are labels correct?

4. **Review metrics per class**
   - Look at per-class mAP in terminal output
   - Some classes might need more data

### Common Issues:

| Problem | Solution |
|---------|----------|
| High train loss, high val loss | Train longer, use larger model |
| Low train loss, high val loss | Overfitting - reduce augmentation, add regularization |
| mAP not improving | Check data quality, try different hyperparameters |
| One class performing poorly | Need more samples of that class |
| Many false positives | Increase confidence threshold, train longer |
| Many false negatives | Lower confidence threshold, check if objects are too small |

---

## 📦 Expected File Sizes

With ~18,000 training images, expect:

| File | Size |
|------|------|
| best.pt | ~45-55 MB (yolo12l) |
| last.pt | ~45-55 MB |
| epoch*.pt | ~45-55 MB each |
| results.csv | 10-50 KB |
| Each plot | 50-500 KB |
| Total output | ~200-300 MB |

---

## 🎯 Quick Quality Checklist

After training, verify:

- [ ] `best.pt` exists in `weights/` folder
- [ ] `results.csv` has all epochs (150 lines if 150 epochs)
- [ ] `results.png` shows decreasing losses
- [ ] mAP50-95 > 0.50 (good), > 0.70 (excellent)
- [ ] All 9 plots are generated
- [ ] Confusion matrix shows high diagonal values
- [ ] Per-class performance is balanced
- [ ] `args.yaml` contains your training config

---

## 📚 Additional Resources

- **Ultralytics Docs:** https://docs.ultralytics.com/
- **Metrics Guide:** https://docs.ultralytics.com/guides/yolo-performance-metrics/
- **Visualizations:** https://docs.ultralytics.com/guides/visualization-tips/

---

**All outputs are automatically saved and verified by the training script! 🎉**

