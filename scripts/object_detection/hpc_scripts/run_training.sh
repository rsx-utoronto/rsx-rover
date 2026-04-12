#!/bin/bash

#SBATCH --job-name=yolo_object_detection
#SBATCH --time=72:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --gres=gpu:h100:1
#SBATCH --output=logs/yolo_training_%j.out
#SBATCH --error=logs/yolo_training_%j.err

# ============================================================================
# YOLO Object Detection Training - Killarney HPC
# ============================================================================

echo "============================================================"
echo "Job $SLURM_JOB_ID started at $(date)"
echo "Node: $SLURM_JOB_NODELIST"
echo "============================================================"

# Base directory
BASE_DIR=~/projects/aip-yuweilai/jackyli/RSX_Object_Detection
cd $BASE_DIR

# Setup
mkdir -p logs runs/detect
export CUDA_VISIBLE_DEVICES=0
export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK
export PYTHONPATH="${PYTHONPATH}:$BASE_DIR"

# Activate environment
source $BASE_DIR/yolo_env/bin/activate
echo "Python: $(which python) | $(python --version)"
echo "Working directory: $(pwd)"

# ============================================================================
# EXTRACT DATASET
# ============================================================================

cd $SLURM_TMPDIR || exit 1
mkdir -p yolo_data && cd yolo_data

DATASET_TAR="$BASE_DIR/comp_data_sorted.tar"

if [ ! -f "$DATASET_TAR" ]; then
    echo "❌ Error: Dataset not found at $DATASET_TAR"
    exit 1
fi

echo ""
echo "Extracting dataset..."
echo "From: $DATASET_TAR"
cp "$DATASET_TAR" . && tar -xf comp_data_sorted.tar

# Navigate to data directory
[ -d "comp_data_sorted" ] && cd comp_data_sorted || echo "Extracted to current directory"
DATA_DIR="$(pwd)"

# Verify structure
for split in train val test; do
    if [ ! -d "$split/images" ] || [ ! -d "$split/labels" ]; then
        echo "❌ Error: Missing $split/images or $split/labels"
        exit 1
    fi
    echo "✅ $split: $(ls $split/images | wc -l) images"
done

# ============================================================================
# TRAINING
# ============================================================================

# Create data.yaml
cp "$BASE_DIR/hpc_data.yaml" "$DATA_DIR/data.yaml"

# Training parameters
MODEL="yolo12l.pt"
EPOCHS=150
BATCH_SIZE=16
IMG_SIZE=640
WORKERS=8
PATIENCE=20
RUN_NAME="yolo12l_hpc_run_$(date +%Y%m%d_%H%M%S)"
PROJECT_DIR="$BASE_DIR/runs/detect"

echo ""
echo "============================================================"
echo "TRAINING CONFIGURATION"
echo "============================================================"
echo "Model: $MODEL | Epochs: $EPOCHS | Batch: $BATCH_SIZE"
echo "Image Size: $IMG_SIZE | Workers: $WORKERS"
echo "Results: $PROJECT_DIR/$RUN_NAME"
echo "============================================================"
echo ""

# Run training
python "$BASE_DIR/scripts/object_detection/hpc_scripts/train_hpc.py" \
    --data "$YAML_FILE" \
    --model "$MODEL" \
    --epochs $EPOCHS \
    --batch $BATCH_SIZE \
    --imgsz $IMG_SIZE \
    --device "0" \
    --workers $WORKERS \
    --patience $PATIENCE \
    --name "$RUN_NAME" \
    --project "$PROJECT_DIR"

TRAIN_EXIT_CODE=$?

# ============================================================================
# RESULTS
# ============================================================================

echo ""
echo "============================================================"

if [ $TRAIN_EXIT_CODE -eq 0 ]; then
    echo "✅ TRAINING COMPLETED SUCCESSFULLY"
    echo "============================================================"
    
    RESULTS_DIR="$PROJECT_DIR/$RUN_NAME"
    
    # Quick verification
    [ -d "$RESULTS_DIR/weights" ] && echo "✅ Weights saved: $RESULTS_DIR/weights/"
    [ -f "$RESULTS_DIR/results.csv" ] && echo "✅ Metrics saved: $RESULTS_DIR/results.csv"
    
    # Count outputs
    if [ -d "$RESULTS_DIR" ]; then
        plot_count=$(ls "$RESULTS_DIR"/*.png "$RESULTS_DIR"/*.jpg 2>/dev/null | wc -l)
        total_size=$(du -sh "$RESULTS_DIR" | cut -f1)
        echo "✅ Generated $plot_count plots"
        echo "📊 Total output: $total_size"
    fi
    
    echo ""
    echo "Usage:"
    echo "  model = YOLO('$RESULTS_DIR/weights/best.pt')"
    echo "  results = model.predict('image.jpg')"
    
else
    echo "❌ TRAINING FAILED (exit code: $TRAIN_EXIT_CODE)"
    echo "============================================================"
    echo "Check logs:"
    echo "  - logs/yolo_training_$SLURM_JOB_ID.out"
    echo "  - logs/yolo_training_$SLURM_JOB_ID.err"
    exit 1
fi

# ============================================================================
# CLEANUP
# ============================================================================

cd $BASE_DIR
rm -rf $SLURM_TMPDIR/yolo_data 2>/dev/null

echo ""
echo "============================================================"
echo "Job completed at $(date)"
echo "Results: $PROJECT_DIR/$RUN_NAME"
echo "============================================================"
