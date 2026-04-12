import cv2, torch
from ultralytics import YOLO
import os

# --- 1) Device & model ---
device = 0 if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model_v1.pt")
model = YOLO(path)
model.to(device)
HALF = torch.cuda.is_available()  # use half precision if on GPU

# --- 2) Classes (choose one of the two) ---
# Option A: use your manual list (must match training order)
# CLASSES = ['waterbottle', 'mallet', 'hammer']
# Option B: read from model (recommended if trained with dataset YAML)
CLASSES = list(model.names.values())

# --- 3) Video source & thresholds ---
cap = cv2.VideoCapture(1)  # or a video path
CONF_THRES = 0.5           # confidence threshold
IMG_SIZE = 640             # inference size; match training if possible

while True:
    ok, frame_bgr = cap.read()
    if not ok:
        break

    # OpenCV -> RGB for Ultralytics
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    # --- 4) Inference (Ultralytics handles preprocess + NMS) ---
    results = model.predict(
        source=frame_rgb,  # numpy image
        imgsz=IMG_SIZE,
        conf=CONF_THRES,
        device=device,
        half=HALF,
        verbose=False
    )

    # --- 5) Draw detections ---
    r = results[0]
    if r.boxes is not None and len(r.boxes) > 0:
        boxes = r.boxes.xyxy.detach().cpu().numpy()        # [N, 4]
        confs = r.boxes.conf.detach().cpu().numpy()        # [N]
        clses = r.boxes.cls.detach().cpu().numpy().astype(int)  # [N]

        for (x1, y1, x2, y2), s, c in zip(boxes, confs, clses):
            if s < CONF_THRES:
                continue
            name = CLASSES[c] if c < len(CLASSES) else str(c)
            cv2.rectangle(frame_bgr, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"{name} {s:.2f}",
                        (int(x1), max(0, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("YOLOv12 (PyTorch) + OpenCV", frame_bgr)
    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()