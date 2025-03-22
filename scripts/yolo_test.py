import cv2
import numpy as np
import torch
from ultralytics import YOLO
import os

# NOTE: Needs to be ran at least once online to cache the MiDaS model

# Initialize MiDaS depth estimator
midas = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True)
midas.eval()
midas.to('cuda' if torch.cuda.is_available() else 'cpu')

# Load YOLO model with explicit weights
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, 'best.pt')

# Verify model file exists
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model weights not found at: {model_path}")

model = YOLO(model_path)  # Load custom trained weights
print(f"Successfully loaded weights from: {model_path}")

# Camera configuration
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 384)

def process_depth(frame):
    """Generate depth map with dimension validation"""
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (384, 384))
    img_tensor = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float()
    
    if torch.cuda.is_available():
        img_tensor = img_tensor.to('cuda')
    
    with torch.no_grad():
        depth = midas(img_tensor)
    
    depth = torch.nn.functional.interpolate(
        depth.unsqueeze(1),
        size=frame.shape[:2],
        mode="bilinear",
        align_corners=False
    ).squeeze().cpu().numpy()
    
    return cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)

def enhance_detection(frame, depth_map):
    """Optimized image enhancement for object detection"""
    # Convert to float for processing
    frame_float = frame.astype(np.float32) / 255.0
    
    # CLAHE on luminance channel
    lab = cv2.cvtColor(frame_float, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    l_clahe = clahe.apply((l * 255).astype(np.uint8)).astype(np.float32) / 255.0
    
    # Merge enhanced channels
    enhanced_lab = cv2.merge((l_clahe, a, b))
    enhanced = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
    
    # Depth-aware sharpening
    sharpened = cv2.filter2D(enhanced, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]))
    
    # Combine with original using depth weights
    depth_weight = np.clip(depth_map * 1.5, 0, 1)[..., np.newaxis]
    processed = enhanced * 0.7 + sharpened * 0.3 * depth_weight
    
    return (np.clip(processed, 0, 1) * 255).astype(np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Process frame
    depth_map = process_depth(frame)
    processed_frame = enhance_detection(frame, depth_map)
    
    # Run YOLO inference
    results = model(processed_frame, imgsz=640, conf=0.4)
    
    # Draw detections
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = f"{model.names[int(box.cls[0])]} {box.conf[0]:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("Depth-Optimized Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()