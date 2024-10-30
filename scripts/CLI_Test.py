from ultralytics import YOLO


model = YOLO('yolov8s.pt')  # Load model


results = model.train(data=r'/home/garvish/proj/Plastic Bottle Image Dataset/data.yaml', epochs=100, batch=16)