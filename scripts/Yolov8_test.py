from ultralytics import YOLO


model = YOLO('yolov8s.pt')  # Load model


results = model.train(data=r'/home/rsx-base/rsx-rover/scripts/nov16.yaml', epochs=300, batch=16, device=0)
#originally: was epochs 200 , bacth 16