from ultralytics import YOLO


model = YOLO('yolov8n.pt')  # Load model


results = model.train(data=r'/home/garvish/proj/Plastic Bottle Image Dataset/data.yaml', epochs=100, batch=8, device='cuda')