from ultralytics import YOLO


model = YOLO('yolov8s.pt')  # Load model


results = model.train(data=r'/home/rsx-base/rover_ws/src/rsx-rover/scripts/datafinal.yaml', epochs=300, batch=16, device=0)