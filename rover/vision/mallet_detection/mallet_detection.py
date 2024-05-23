from ultralytics import YOLO

model = YOLO(r'/home/garvish/rover_ws/src/rsx-rover/rover/vision/mallet_detection/mallet_detection.pt')

while True:
    # res = model.predict(source=r'/home/garvish/rover_ws/src/rsx-rover/rover/vision/mallet_detection/rgb_2.png', show=True)
    res = model.predict(source=0, show=True)