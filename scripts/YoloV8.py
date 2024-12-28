from ultralytics import YOLO


model = YOLO('yolov8s.pt')  # Load model


results = model.train(data=r'/home/rsx-base/rsx-rover/scripts/datafinal.yaml', epochs=200, batch=16, device=0)