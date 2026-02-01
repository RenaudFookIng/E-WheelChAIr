import cv2
import torch
import numpy as np
import sys
import os
import time
import math
import threading
import queue

# ===== PATH DEPTH ANYTHING =====
script_dir = os.path.dirname(os.path.abspath(__file__))
depth_anything_path = os.path.join(script_dir, 'Depth-Anything-V2')
sys.path.append(depth_anything_path)

from torchvision.transforms import Compose
from ultralytics import YOLO
from depth_anything_v2.dpt import DepthAnythingV2
from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet

# ================= CONFIG =================
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
INPUT_SIZE = 256
TARGET_FPS = 60

FRAME_SKIP_YOLO = 3
FRAME_SKIP_DEPTH = 2

CRITICAL_REL = 0.8
WARNING_REL = 0.5

CROP_PERCENT = 0.15

# ================= LOAD MODELS =================
print("🚀 Loading models...")

yolo_model = YOLO('yolov8n.pt')
yolo_model.fuse()

depth_model = DepthAnythingV2(
    encoder='vits',
    features=64,
    out_channels=[48, 96, 192, 384]
).to(DEVICE).eval()

pth_path = "depth_anything_v2_vits.pth"
if os.path.exists(pth_path):
    depth_model.load_state_dict(torch.load(pth_path, map_location=DEVICE))
else:
    url = "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth"
    depth_model.load_state_dict(torch.hub.load_state_dict_from_url(url, map_location=DEVICE))

transform = Compose([
    Resize(INPUT_SIZE, INPUT_SIZE, keep_aspect_ratio=True, ensure_multiple_of=14),
    NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    PrepareForNet()
])

# ================= GLOBAL SHARED DATA =================
latest_frame = None
latest_depth = None
latest_yolo = []

lock = threading.Lock()
running = True

# ================= CAMERA THREAD =================
def camera_thread():
    global latest_frame, running

    cap0 = cv2.VideoCapture(1)
    cap1 = cv2.VideoCapture(0)

    cap0.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    cap0.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap1.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    while running:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        if not ret0 or not ret1:
            continue

        frame0 = cv2.flip(frame0, 0)
        frame1 = cv2.flip(frame1, 0)

        frame = merge_frames(frame0, frame1)

        with lock:
            latest_frame = frame

    cap0.release()
    cap1.release()

# ================= MERGE CAMERAS =================
def merge_frames(frame0, frame1):
    height = 720
    frame0 = cv2.resize(frame0, (int(frame0.shape[1]*height/frame0.shape[0]), height))
    frame1 = cv2.resize(frame1, (int(frame1.shape[1]*height/frame1.shape[0]), height))

    crop = int(frame0.shape[1] * CROP_PERCENT)

    frame0 = frame0[:, crop:-crop]
    frame1 = frame1[:, crop:-crop]

    return np.hstack((frame0, frame1))

# ================= YOLO THREAD =================
def yolo_thread():
    global latest_frame, latest_yolo, running
    count = 0

    while running:
        if latest_frame is None:
            time.sleep(0.002)
            continue

        count += 1
        if count % FRAME_SKIP_YOLO != 0:
            time.sleep(0.002)
            continue

        with lock:
            frame = latest_frame.copy()

        results = yolo_model.predict(frame, verbose=False)
        latest_yolo = results

# ================= DEPTH THREAD =================
def depth_thread():
    global latest_frame, latest_depth, running
    count = 0

    while running:
        if latest_frame is None:
            time.sleep(0.002)
            continue

        count += 1
        if count % FRAME_SKIP_DEPTH != 0:
            time.sleep(0.002)
            continue

        with lock:
            frame = latest_frame.copy()

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
        frame_tensor = transform({'image': frame_rgb})['image']
        frame_tensor = torch.from_numpy(frame_tensor).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            depth = depth_model(frame_tensor)

        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(0),
            size=(frame.shape[0]//2, frame.shape[1]//2),
            mode='nearest'
        )

        depth = cv2.resize(depth.squeeze().cpu().numpy(), (frame.shape[1], frame.shape[0]))

        depth_norm = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)
        latest_depth = depth_norm

# ================= DISPLAY THREAD =================
def display_thread():
    global latest_frame, latest_depth, latest_yolo, running

    fps_timer = time.time()
    fps_count = 0
    fps_value = 0

    while running:
        if latest_frame is None:
            time.sleep(0.001)
            continue

        with lock:
            frame = latest_frame.copy()

        H, W, _ = frame.shape

        # Draw YOLO
        if latest_yolo:
            for result in latest_yolo:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = yolo_model.names[int(box.cls[0])]
                    conf = float(box.conf[0])

                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                    color = (0, 255, 0)
                    zone = "SAFE"

                    if latest_depth is not None and 0 <= cy < H and 0 <= cx < W:
                        dist_rel = latest_depth[cy, cx]

                        if dist_rel > CRITICAL_REL:
                            zone = "DANGER"
                            color = (0, 0, 255)
                        elif dist_rel > WARNING_REL:
                            zone = "WARNING"
                            color = (0, 165, 255)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"{label} {zone}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        fps_count += 1
        if time.time() - fps_timer >= 1:
            fps_value = fps_count
            fps_count = 0
            fps_timer = time.time()

        cv2.putText(frame, f"FPS Display: {fps_value}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        cv2.imshow("🚗 E-WheelChAIr Vision ULTRA FAST", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False

# ================= MAIN =================
threads = [
    threading.Thread(target=camera_thread, daemon=True),
    threading.Thread(target=yolo_thread, daemon=True),
    threading.Thread(target=depth_thread, daemon=True),
    threading.Thread(target=display_thread, daemon=True)
]

for t in threads:
    t.start()

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    running = False

cv2.destroyAllWindows()
print("✅ Stopped cleanly")
