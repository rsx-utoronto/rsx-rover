import cv2
import numpy as np
import torch 
from ultralytics import YOLO
# Load the model

#model = model.load("yolov8s.pt")
model=YOLO("best.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO model inference
    results = model(frame)

    # Loop through results and draw bounding boxes
    for result in results:
        boxes = result.boxes  # Each box is an object detected

        for box in boxes:
            # Extract box coordinates, confidence, and class
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            conf = box.conf[0]  # Confidence score
            cls = box.cls[0]    # Class label

            # Draw bounding box and label on the frame
            label = f"{model.names[int(cls)]}: {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with bounding boxes
    cv2.imshow("YOLO Detection", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
