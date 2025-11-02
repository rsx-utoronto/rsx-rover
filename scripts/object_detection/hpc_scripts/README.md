# YOLO Training on Killarney HPC

This directory contains scripts for training YOLO object detection models on the Killarney HPC cluster.

## Files

- `train.py` - Local training script
- `train_hpc.py` - HPC-optimized training script with more options
- `hpc_data.yaml` - Dataset configuration for HPC
- `run_training.sh` - SLURM job submission script
- `README.md` - This file

## Setup Instructions

### 1. Prepare Your Dataset

First, create a tar.gz archive of your training data:

```bash
# Navigate to the parent directory of comp_data_sorted
cd /path/to/your/data/

# Create tar.gz file
tar -czf comp_data_sorted.tar.gz comp_data_sorted/

# The structure should be:
# comp_data_sorted/
#   ├── train/
#   │   ├── images/
#   │   └── labels/
#   ├── val/
#   │   ├── images/
#   │   └── labels/
#   └── test/
#       ├── images/
#       └── labels/
```

### 2. Upload to HPC

Transfer your files to Killarney:

```bash
# Create directories on HPC
ssh killarney.scinet.utoronto.ca
mkdir -p ~/yolo_project/data
mkdir -p ~/yolo_project/scripts/object_detection/hpc_scripts

# Upload dataset (from your local machine)
scp comp_data_sorted.tar.gz killarney:~/yolo_project/data/

# Upload scripts
scp -r scripts/object_detection/hpc_scripts/* killarney:~/yolo_project/scripts/object_detection/hpc_scripts/
```

### 3. Set Up Python Environment

On Killarney:

```bash
# Load Python module (adjust version if needed)
module load python/3.10

# Create virtual environment
python -m venv ~/yolo_env

# Activate environment
source ~/yolo_env/bin/activate

# Install required packages
pip install --upgrade pip
pip install ultralytics opencv-python torch torchvision
```

### 4. Prepare the Job Script

Edit `run_training.sh` if needed:

```bash
cd ~/yolo_project/scripts/object_detection/hpc_scripts

# Make script executable
chmod +x run_training.sh

# Edit paths in the script if your setup differs
nano run_training.sh
```

**Important lines to check in `run_training.sh`:**
- Line 31: Virtual environment path (`source $HOME/yolo_env/bin/activate`)
- Line 56: Dataset tar file location (`$SLURM_SUBMIT_DIR/data/comp_data_sorted.tar.gz`)
- Line 130-137: Training parameters (model, epochs, batch size, etc.)

### 5. Submit the Job

```bash
# Navigate to your project directory
cd ~/yolo_project

# Create logs directory
mkdir -p logs

# Submit the job
sbatch scripts/object_detection/hpc_scripts/run_training.sh

# Check job status
squeue -u $USER

# View output in real-time (replace JOBID with your actual job ID)
tail -f logs/yolo_training_JOBID.out
```

## Training Parameters

You can modify these parameters in `run_training.sh`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MODEL` | yolo11l.pt | Model size (n/s/m/l/x) |
| `EPOCHS` | 100 | Number of training epochs |
| `BATCH_SIZE` | 16 | Batch size (adjust for GPU memory) |
| `IMG_SIZE` | 640 | Input image size |
| `WORKERS` | 8 | Number of data loading workers |
| `PATIENCE` | 20 | Early stopping patience |

### Model Options

- `yolo12n.pt` - Nano (fastest, least accurate)
- `yolo12s.pt` - Small
- `yolo12m.pt` - Medium
- `yolo12l.pt` - Large (recommended)
- `yolo12x.pt` - Extra Large (best accuracy, slower)

## Monitoring Training

### Check Job Status

```bash
# View your jobs
squeue -u $USER

# Cancel a job
scancel JOBID

# View job details
scontrol show job JOBID
```

### View Logs

```bash
# Real-time output
tail -f logs/yolo_training_JOBID.out

# Error log
tail -f logs/yolo_training_JOBID.err
```

### Training Progress

The training script will display:
- Epoch progress
- Loss values (box, cls, dfl)
- Metrics (precision, recall, mAP)
- Time per epoch
- ETA for completion

## Results

After training completes, results will be in:

```
~/yolo_project/runs/detect/yolo12l_hpc_run_TIMESTAMP/
├── weights/
│   ├── best.pt         # Best model weights (highest mAP)
│   └── last.pt         # Last epoch weights
├── results.csv         # Training metrics per epoch
├── confusion_matrix.png
├── F1_curve.png
├── P_curve.png
├── R_curve.png
├── PR_curve.png
└── ...other plots
```

## Using Your Trained Model

After training, you can use the model:

```python
from ultralytics import YOLO

# Load your trained model
model = YOLO('runs/detect/yolo12l_hpc_run_TIMESTAMP/weights/best.pt')

# Run inference
results = model.predict('path/to/image.jpg', save=True)

# Or on a folder
results = model.predict('path/to/images/', save=True)
```

## Troubleshooting

### Common Issues

**1. Dataset tar file not found**
- Check the path in `run_training.sh` line 56
- Ensure you uploaded the tar.gz file correctly

**2. GPU out of memory**
- Reduce `BATCH_SIZE` in `run_training.sh`
- Try: 16 → 8 → 4

**3. Virtual environment not found**
- Update path in `run_training.sh` line 31
- Recreate environment if needed

**4. Training too slow**
- Use smaller model (yolo12s.pt or yolo12m.pt)
- Reduce image size: 640 → 512
- Ensure using GPU (check CUDA_VISIBLE_DEVICES)

**5. Job times out**
- Increase time limit: `#SBATCH --time=72:00:00`
- Reduce epochs or enable resume

### Resume Training

If training times out, you can resume:

```bash
# Edit run_training.sh and change line:
# Add --resume flag to training command

# Or manually resume:
python train_hpc.py --data path/to/data.yaml --resume
```

## Best Practices

1. **Start small**: Test with yolo12s.pt and 10 epochs first
2. **Monitor GPU usage**: `nvidia-smi` in your output logs
3. **Save checkpoints**: The script saves every 10 epochs
4. **Use validation**: The script validates on test set after training
5. **Batch size**: Start with 16, adjust based on GPU memory

## Performance Tips

- **H100 GPU**: Can handle batch size 32-64 for yolo12l
- **Data loading**: 8 workers is optimal for most cases
- **Mixed precision**: Enabled by default (AMP) for faster training
- **Cache**: Disabled on HPC to save memory

## Expected Training Time

On H100 GPU with 18,000 images:

| Model | Batch Size | Time per Epoch | 100 Epochs |
|-------|------------|----------------|------------|
| yolo12s | 32 | ~5 min | ~8 hours |
| yolo12m | 24 | ~8 min | ~13 hours |
| yolo12l | 16 | ~12 min | ~20 hours |
| yolo12x | 8 | ~20 min | ~33 hours |

## Support

For HPC-specific issues:
- SciNet documentation: https://docs.scinet.utoronto.ca/
- Killarney guide: https://docs.scinet.utoronto.ca/index.php/Killarney

For YOLO-specific issues:
- Ultralytics docs: https://docs.ultralytics.com/
- GitHub: https://github.com/ultralytics/ultralytics

---

**Good luck with your training! 🚀**

