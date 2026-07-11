# HPC Setup - Directory Structure

## 📁 Your Directory Structure

Everything is located in:
```
~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/
```

### Expected Structure:
```
RSX_Object_Detection/
├── comp_data_sorted.tar          # Your dataset (uncompressed tar)
├── yolo12l.pt                     # Model weights (downloaded automatically)
├── yolo_env/                      # Python virtual environment
│   ├── bin/
│   │   └── activate
│   └── ...
├── scripts/
│   └── object_detection/
│       └── hpc_scripts/
│           ├── run_training.sh    # Main SLURM script
│           ├── train_hpc.py       # Training script
│           ├── hpc_data.yaml      # Dataset config
│           └── ...
├── logs/                          # Created automatically
│   ├── yolo_training_*.out
│   └── yolo_training_*.err
└── runs/                          # Created automatically
    └── detect/
        └── yolo12l_hpc_run_*/     # Training results
            ├── weights/
            │   ├── best.pt
            │   └── last.pt
            ├── results.csv
            └── *.png
```

## ✅ Pre-requisites Checklist

Make sure you have all these in `~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/`:

- [ ] `comp_data_sorted.tar` - Your dataset tar file
- [ ] `yolo_env/` - Virtual environment with ultralytics installed
- [ ] `scripts/object_detection/hpc_scripts/` - All training scripts
- [ ] `yolo12l.pt` will be downloaded automatically by ultralytics

## 🚀 Quick Start

### 1. Verify Your Setup

```bash
ssh killarney
cd ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection

# Check files exist
ls -lh comp_data_sorted.tar
ls -d yolo_env/
ls scripts/object_detection/hpc_scripts/run_training.sh

# Test virtual environment
source yolo_env/bin/activate
python --version
python -c "import ultralytics; print(ultralytics.__version__)"
deactivate
```

### 2. Submit Training Job

```bash
cd ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection

# Create logs directory
mkdir -p logs

# Submit job
sbatch scripts/object_detection/hpc_scripts/run_training.sh

# Check status
squeue -u $USER

# Monitor output
tail -f logs/yolo_training_JOBID.out
```

## 📝 Important Paths in run_training.sh

All paths now use `BASE_DIR`:
```bash
BASE_DIR=~/projects/aip-yuweilai/jackyli/RSX_Object_Detection

# These are set automatically:
DATASET_TAR="$BASE_DIR/comp_data_sorted.tar"
VENV="$BASE_DIR/yolo_env/bin/activate"
SCRIPTS="$BASE_DIR/scripts/object_detection/hpc_scripts/"
RESULTS="$BASE_DIR/runs/detect/"
LOGS="$BASE_DIR/logs/"
```

## 🔧 Customizing Training

Edit these lines in `run_training.sh`:

```bash
# Line 78-83: Training parameters
MODEL="yolo12l.pt"        # Model size
EPOCHS=150                # Number of epochs
BATCH_SIZE=16             # Batch size
IMG_SIZE=640              # Image size
WORKERS=8                 # Data loading workers
PATIENCE=20               # Early stopping patience
```

## 📊 Where Results Are Saved

After training completes:
```
~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/runs/detect/yolo12l_hpc_run_TIMESTAMP/
├── weights/
│   ├── best.pt          ← Use this for inference
│   ├── last.pt
│   └── epoch*.pt
├── results.csv          ← Training metrics
└── *.png                ← Plots and visualizations
```

## 🐛 Troubleshooting

### Dataset not found
```bash
# Check if tar file exists
ls -lh ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/comp_data_sorted.tar

# Verify it's a valid tar
tar -tf ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/comp_data_sorted.tar | head
```

### Python environment issues
```bash
# Recreate if needed
cd ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection
rm -rf yolo_env
python3 -m venv yolo_env
source yolo_env/bin/activate
pip install --upgrade pip
pip install ultralytics opencv-python torch torchvision
```

### Model weights not found
The model weights (`yolo12l.pt`) will be downloaded automatically by Ultralytics on first run. No action needed.

### Logs location
```bash
# Output log
tail -f ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/logs/yolo_training_JOBID.out

# Error log (if any)
tail -f ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection/logs/yolo_training_JOBID.err
```

## 📞 Common Commands

```bash
# Submit job
cd ~/projects/aip-yuweilai/jackyli/RSX_Object_Detection
sbatch scripts/object_detection/hpc_scripts/run_training.sh

# Check job status
squeue -u $USER

# Cancel job
scancel JOBID

# View output
tail -f logs/yolo_training_JOBID.out

# Check results
ls -lh runs/detect/

# List all runs
ls -lt runs/detect/ | head
```

## 🎯 Final Checklist Before Submitting

- [ ] You're in the correct directory: `~/projects/aip-yuweilai/jackyli/RSX_Object_Detection`
- [ ] `comp_data_sorted.tar` exists (check with `ls -lh comp_data_sorted.tar`)
- [ ] Virtual environment works (test with `source yolo_env/bin/activate`)
- [ ] Logs directory exists (`mkdir -p logs`)
- [ ] You've reviewed the training parameters in `run_training.sh`

Then run:
```bash
sbatch scripts/object_detection/hpc_scripts/run_training.sh
```

---

**Everything is configured to use your directory structure! 🚀**

