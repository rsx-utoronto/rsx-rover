from ultralytics import YOLO
import argparse

def main():
    # --- command line arguments ---
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="Path to data.yaml")
    ap.add_argument("--epochs", type=int, default=30, help="Number of epochs to train")
    ap.add_argument("--batch", type=int, default=8, help="Batch size")
    ap.add_argument("--imgsz", type=int, default=640, help="Image size")
    ap.add_argument("--device", default="0", help='Device: "0" for GPU or "cpu"')
    ap.add_argument("--name", default="yolov12l_exp", help="Run name for saving results")
    args = ap.parse_args()

    # --- print info ---
    print("===================================")
    print("YOLOv12l Training")
    print("===================================")
    print(f"Model : yolo12l.pt")
    print(f"Data  : {args.data}")
    print(f"Epochs: {args.epochs}")
    print(f"Batch : {args.batch}")
    print(f"ImgSz : {args.imgsz}")
    print(f"Device: {args.device}")
    print("===================================\n")

    # --- load and train model ---
    model = YOLO("yolo12l.pt")

    model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        name=args.name,
        project="runs",
    )

    # --- validate ---
    model.val(data=args.data, device=args.device)

    # --- (optional) run prediction on val images ---
    print("\n✅ Training complete! Results are saved in the 'runs/' folder.")
    print("Use model.predict(source='path/to/image_or_folder', save=True) to test your model.\n")

if __name__ == "__main__":
    main()