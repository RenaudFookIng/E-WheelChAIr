import cv2
import torch
import numpy as np
import sys
import os
import socket
import time
import math
from PIL import Image
import torch.nn.functional as F 
from torchvision.transforms import Compose

# ================== CPU OPTIMIZATION (MAC) ==================
torch.set_num_threads(4)
DEVICE = 'cpu'

# ================= Network Configuration ====================
RASPBERRY_IP = '192.168.50.2'
RASPBERRY_PORT = 5000

# ================= Path Fix ================================
script_dir = os.path.dirname(os.path.abspath(__file__))
da_v2_path = os.path.join(script_dir, 'Depth-Anything-V2')
sys.path.append(da_v2_path)

# ================= Imports ================================
from ultralytics import YOLO
from depth_anything_v2.dpt import DepthAnythingV2
from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet

# ================= PERFORMANCE CONFIG ======================
INPUT_SIZE = 192  
DEPTH_MODEL_NAME = 'depth_anything_v2_vits'
INTERPOLATION_MODE = 'bicubic'

FRAME_SKIP_YOLO = 3
FRAME_SKIP_DEPTH = 2
DISPLAY_SKIP = 2

frame_count = 0
last_yolo_results = None
last_depth_normalized = None

# ================= Load YOLO ===============================
print("Loading YOLO...")
yolo_model = YOLO('yolov8n.pt')

# ================= Load Depth Anything =====================
print("Loading Depth Anything...")
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]}
}
encoder = DEPTH_MODEL_NAME.split('_')[-1]
depth_model = DepthAnythingV2(**model_configs[encoder]).to(DEVICE).eval()

pth_path = f'{DEPTH_MODEL_NAME}.pth'
if os.path.exists(pth_path):
    depth_model.load_state_dict(torch.load(pth_path, map_location=DEVICE))
else:
    url = "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true"
    depth_model.load_state_dict(torch.hub.load_state_dict_from_url(url, map_location=DEVICE))

# ================= Transform ===============================
transform = Compose([
    Resize(width=INPUT_SIZE, height=INPUT_SIZE, keep_aspect_ratio=True, ensure_multiple_of=14),
    NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    PrepareForNet()
])

# ================= Cameras ================================
cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(1)

if not cap0.isOpened() or not cap1.isOpened():
    print("❌ Cameras not available")
    sys.exit(1)

# ================= Distance Calibration ===================
known_relative_vals = [0.10, 0.40, 0.90]
known_real_meters   = [5.0,  3.0,  1.0]

IGNORE_CLOSE_DIST = 1.2
IGNORE_FAR_DIST   = 4.5

# ================= Networking =============================
sock = None
connected = False

def connect_to_pi():
    global sock, connected
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        sock.connect((RASPBERRY_IP, RASPBERRY_PORT))
        connected = True
        print("✅ Connected to Raspberry Pi")
    except:
        connected = False

connect_to_pi()

# ================= FPS ================================
fps_timer = time.time()
fps_counter = 0
fps_value = 0

# ================= MAIN LOOP ===========================
try:
    while True:
        frame_count += 1
        
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        if not ret0 or not ret1:
            break
        
        frame = np.hstack((frame0, frame1))
        H, W, _ = frame.shape
        
        # ================= DEPTH =========================
        if frame_count % FRAME_SKIP_DEPTH == 0 or last_depth_normalized is None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
            tensor = transform({'image': rgb})['image']
            tensor = torch.from_numpy(tensor).unsqueeze(0).to(DEVICE)
            
            with torch.no_grad():
                depth_tensor = depth_model(tensor)

            depth_tensor = F.interpolate(
                depth_tensor.unsqueeze(0),
                size=(H, W),
                mode=INTERPOLATION_MODE,
                align_corners=None
            ).squeeze()

            depth = depth_tensor.cpu().numpy()
            last_depth_normalized = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)

        depth_normalized = last_depth_normalized
        
        # ================= YOLO ==========================
        if frame_count % FRAME_SKIP_YOLO == 0 or last_yolo_results is None:
            small_frame = cv2.resize(frame, (640, 360))
            last_yolo_results = yolo_model.predict(small_frame, verbose=False)

        yolo_results = last_yolo_results
        
        # ================= DISPLAY ======================
        depth_colormap = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)

        closest_obj_data = None
        min_distance_found = 999.0
        
        for result in yolo_results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(i * (W / 640)) for i in box.xyxy[0]]
                label = yolo_model.names[int(box.cls[0])]
                confidence = float(box.conf[0])
                
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                if cx < 0 or cy < 0 or cx >= W or cy >= H:
                    continue
                
                dist_rel = depth_normalized[cy, cx]
                dist_m = np.interp(dist_rel, known_relative_vals, known_real_meters)
                
                # Ignore zone
                if dist_m < IGNORE_CLOSE_DIST or dist_m > IGNORE_FAR_DIST:
                    cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (128,128,128), 1)
                    continue
                
                FOV = 170.0
                angle_deg = (cx - W/2) / (W/2) * (FOV / 2)
                lateral_offset = dist_m * math.tan(math.radians(angle_deg))
                
                if dist_m < min_distance_found:
                    min_distance_found = dist_m
                    
                    seen_by_left = 1 if cx < (W/2) else 0
                    seen_by_right = 1 if cx >= (W/2) else 0
                    layer = 1 if dist_m < 1.5 else (2 if dist_m < 3.0 else 3)
                    
                    closest_obj_data = (
                        f"{label},{dist_m:.2f},{lateral_offset:.2f},{angle_deg:.1f},"
                        f"{layer},0.85,{confidence:.2f},{seen_by_left},{seen_by_right}"
                    )
                
                cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (255,255,255), 2)
                cv2.putText(depth_colormap, f"{label} {dist_m:.2f}m",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        # ================= NETWORK SEND ==================
        if closest_obj_data:
            msg = closest_obj_data + "\n"
            if connected:
                try:
                    sock.sendall(msg.encode())
                except:
                    connected = False
            else:
                if int(time.time()) % 2 == 0:
                    connect_to_pi()

        # ================= FPS ===========================
        fps_counter += 1
        if time.time() - fps_timer >= 1:
            fps_value = fps_counter
            fps_counter = 0
            fps_timer = time.time()

        cv2.putText(depth_colormap, f"FPS: {fps_value}", (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        # ================= DISPLAY =======================
        if frame_count % DISPLAY_SKIP == 0:
            cv2.imshow("YOLO + Depth Fusion (Optimized)", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()
    if sock: sock.close()
    print("Program stopped.")
