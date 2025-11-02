# Quick Start Guide - HPC Training

## 🚀 Fast Setup (5 steps)

### 1. Prepare Dataset
```bash
# From your local machine, in the directory containing comp_data_sorted/
bash scripts/object_detection/hpc_scripts/prepare_dataset.sh
```

### 2. Upload to HPC
```bash
# Upload dataset
scp comp_data_sorted.tar.gz killarney:~/yolo_project/data/

# Upload scripts
scp -r scripts/object_detection/hpc_scripts killarney:~/yolo_project/scripts/object_detection/
```

### 3. Setup Environment (on HPC)
```bash
# SSH to Killarney
ssh killarney.scinet.utoronto.ca

# Create environment
module load python/3.10
python -m venv ~/yolo_env
source ~/yolo_env/bin/activate
pip install ultralytics opencv-python torch torchvision
```

### 4. Edit Paths (if needed)
```bash
cd ~/yolo_project
nano scripts/object_detection/hpc_scripts/run_training.sh

# Check these lines:
# Line 31: source $HOME/yolo_env/bin/activate
# Line 56: DATASET_TAR="$SLURM_SUBMIT_DIR/data/comp_data_sorted.tar.gz"
```

### 5. Submit Job
```bash
cd ~/yolo_project
mkdir -p logs
sbatch scripts/object_detection/hpc_scripts/run_training.sh
```

## 📊 Monitor Training

```bash
# Check job status
squeue -u $USER

# View output (replace JOBID)
tail -f logs/yolo_training_JOBID.out

# View errors (if any)
tail -f logs/yolo_training_JOBID.err

# Cancel job
scancel JOBID
```

## 📁 Results Location

```
~/yolo_project/runs/detect/yolo12l_hpc_run_TIMESTAMP/
├── weights/best.pt  ← Use this for inference
├── weights/last.pt
├── results.csv      ← Training metrics
└── *.png            ← Plots
```

## ⚙️ Quick Configuration Changes

**Change model size** (in `run_training.sh`, line 142):
```bash
MODEL="yolo12s.pt"  # Smaller, faster
MODEL="yolo12m.pt"  # Medium
MODEL="yolo12l.pt"  # Large (default, recommended)
MODEL="yolo12x.pt"  # Extra large, best accuracy
```

**Adjust batch size** (line 144):
```bash
BATCH_SIZE=8   # If getting GPU OOM errors
BATCH_SIZE=16  # Default
BATCH_SIZE=32  # If you have H100 and want faster training
```

**Change epochs** (line 143):
```bash
EPOCHS=50   # Quick test
EPOCHS=100  # Default
EPOCHS=150  # More training
```

## 🔧 Common Issues

| Issue | Solution |
|-------|----------|
| "Dataset tar not found" | Check path on line 56 of `run_training.sh` |
| GPU out of memory | Reduce `BATCH_SIZE` to 8 or 4 |
| Training too slow | Use smaller model (yolo12s.pt) |
| Job times out | Increase `--time=72:00:00` on line 3 |
| Can't find venv | Update path on line 31 |

## 📞 Need Help?

Check full documentation in `README.md`

---
**Expected time on H100: ~20 hours for yolo12l with 100 epochs**

