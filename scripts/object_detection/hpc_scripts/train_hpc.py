from ultralytics import YOLO
import argparse
import os
import shutil
from pathlib import Path

def main():
    # --- command line arguments ---
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="Path to data.yaml")
    ap.add_argument("--epochs", type=int, default=100, help="Number of epochs to train")
    ap.add_argument("--batch", type=int, default=16, help="Batch size")
    ap.add_argument("--imgsz", type=int, default=640, help="Image size")
    ap.add_argument("--device", default="0", help='Device: "0" for GPU or "cpu"')
    ap.add_argument("--name", default="yolov12l_hpc", help="Run name for saving results")
    ap.add_argument("--model", default="yolo12l.pt", help="Model to use (yolo12l.pt, yolo12x.pt, etc.)")
    ap.add_argument("--workers", type=int, default=8, help="Number of data loading workers")
    ap.add_argument("--project", default="runs/detect", help="Project folder for saving results")
    ap.add_argument("--patience", type=int, default=20, help="Early stopping patience")
    ap.add_argument("--resume", action="store_true", help="Resume from last checkpoint")
    args = ap.parse_args()

    # --- print info ---
    print("="*60)
    print("YOLO Training on HPC (Killarney)")
    print("="*60)
    print(f"Model   : {args.model}")
    print(f"Data    : {args.data}")
    print(f"Epochs  : {args.epochs}")
    print(f"Batch   : {args.batch}")
    print(f"ImgSz   : {args.imgsz}")
    print(f"Device  : {args.device}")
    print(f"Workers : {args.workers}")
    print(f"Name    : {args.name}")
    print(f"Project : {args.project}")
    print(f"Patience: {args.patience}")
    print("="*60 + "\n")

    # Verify data file exists
    if not os.path.exists(args.data):
        print(f"❌ Error: Data file not found: {args.data}")
        exit(1)
    
    print(f"✅ Data file found: {args.data}\n")

    # --- load and train model ---
    print(f"Loading model: {args.model}")
    model = YOLO(args.model)

    print("Starting training...\n")
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        name=args.name,
        project=args.project,
        workers=args.workers,
        patience=args.patience,
        resume=args.resume,
        save=True,
        save_period=10,  # Save checkpoint every 10 epochs
        cache=False,  # Don't cache images on HPC
        verbose=True,
        plots=True,  # Generate all training plots
        amp=True,  # Use automatic mixed precision
        exist_ok=True,  # Allow overwriting existing directory
        pretrained=True,  # Use pretrained weights
        optimizer='AdamW',  # Optimizer
        cos_lr=True,  # Use cosine learning rate scheduler
        close_mosaic=10,  # Disable mosaic augmentation for final 10 epochs
    )

    print("\n" + "="*60)
    print("Training Complete!")
    print("="*60)

    # --- validate ---
    print("\nRunning validation on test set...")
    metrics = model.val(data=args.data, device=args.device, split='test', save_json=True, plots=True)
    
    print("\n" + "="*60)
    print("VALIDATION METRICS:")
    print("="*60)
    print(f"Precision: {metrics.box.mp:.4f}")
    print(f"Recall:    {metrics.box.mr:.4f}")
    print(f"mAP50:     {metrics.box.map50:.4f}")
    print(f"mAP50-95:  {metrics.box.map:.4f}")
    print("="*60)
    
    # Per-class metrics
    if hasattr(metrics.box, 'ap_class_index'):
        print("\nPer-Class Metrics:")
        print("-" * 60)
        class_names = ['waterbottle', 'mallet', 'hammer']
        for i, class_name in enumerate(class_names):
            if i < len(metrics.box.ap):
                print(f"  {class_name:12s}: mAP50={metrics.box.ap50[i]:.4f}, mAP50-95={metrics.box.ap[i]:.4f}")
        print("-" * 60)

    # Save directory
    save_dir = Path(args.project) / args.name
    
    # Verify and list all output files
    print("\n" + "="*60)
    print("OUTPUT FILES VERIFICATION")
    print("="*60)
    print(f"\nResults saved to: {save_dir}")
    print("\n📁 Directory Structure:")
    
    # Check weights
    weights_dir = save_dir / "weights"
    if weights_dir.exists():
        print("\n✅ Weights:")
        for weight_file in sorted(weights_dir.glob("*.pt")):
            size = weight_file.stat().st_size / (1024*1024)  # Size in MB
            print(f"   - {weight_file.name} ({size:.1f} MB)")
    
    # Check results.csv
    results_csv = save_dir / "results.csv"
    if results_csv.exists():
        print("\n✅ Training Metrics:")
        print(f"   - results.csv ({results_csv.stat().st_size / 1024:.1f} KB)")
        print(f"     Contains: epoch, train/loss, val/loss, metrics, lr, etc.")
    
    # Check plots
    plot_files = [
        "confusion_matrix.png",
        "confusion_matrix_normalized.png",
        "F1_curve.png",
        "P_curve.png",
        "R_curve.png",
        "PR_curve.png",
        "results.png",
        "labels.jpg",
        "labels_correlogram.jpg",
    ]
    
    plots_found = []
    for plot in plot_files:
        plot_path = save_dir / plot
        if plot_path.exists():
            plots_found.append(plot)
    
    if plots_found:
        print("\n✅ Plots & Visualizations:")
        for plot in plots_found:
            print(f"   - {plot}")
    
    # Check for training/validation batch examples
    batch_examples = list(save_dir.glob("train_batch*.jpg")) + list(save_dir.glob("val_batch*.jpg"))
    if batch_examples:
        print("\n✅ Batch Examples:")
        for batch in sorted(batch_examples)[:5]:  # Show first 5
            print(f"   - {batch.name}")
        if len(batch_examples) > 5:
            print(f"   ... and {len(batch_examples) - 5} more")
    
    # Check args.yaml (training configuration)
    args_yaml = save_dir / "args.yaml"
    if args_yaml.exists():
        print("\n✅ Configuration:")
        print(f"   - args.yaml (training configuration backup)")
    
    # Count total files
    all_files = list(save_dir.rglob("*"))
    total_files = len([f for f in all_files if f.is_file()])
    total_size = sum([f.stat().st_size for f in all_files if f.is_file()]) / (1024*1024)  # MB
    
    print(f"\n📊 Total: {total_files} files, {total_size:.1f} MB")
    
    # Summary
    print("\n" + "="*60)
    print("TRAINING SUMMARY")
    print("="*60)
    print(f"Model:      {args.model}")
    print(f"Epochs:     {args.epochs}")
    print(f"Best mAP50: {metrics.box.map50:.4f}")
    print(f"Best mAP:   {metrics.box.map:.4f}")
    print(f"Precision:  {metrics.box.mp:.4f}")
    print(f"Recall:     {metrics.box.mr:.4f}")
    print(f"\nBest model: {save_dir}/weights/best.pt")
    print(f"Last model: {save_dir}/weights/last.pt")
    print("="*60)
    print("\n✅ Training complete! All outputs saved successfully. 🎉\n")

if __name__ == "__main__":
    main()

