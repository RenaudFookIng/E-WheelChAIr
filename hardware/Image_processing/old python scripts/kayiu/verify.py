import cv2
import torch
import torch.hub
import numpy as np
import sys
import os
import socket
import time
import math
from PIL import Image
import torch.nn.functional as F 

# ================= Network Configuration =================
# Ensure Raspberry Pi IP is reachable
RASPBERRY_IP = '192.168.50.2'
RASPBERRY_PORT = 5000
# =========================================================

# --- 1. Dynamic sys.path modification ---
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    da_v2_path = os.path.join(script_dir, 'Depth-Anything-V2')
    if da_v2_path not in sys.path:
        sys.path.append(da_v2_path)
except Exception as e:
    print(f"Failed to modify path: {e}")
    sys.exit(1)

# --- 2. Imports ---
from torchvision.transforms import Compose, ToTensor 
try:
    from ultralytics import YOLO
    from depth_anything_v2.dpt import DepthAnythingV2
    from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet
except ImportError as e:
    print(f"Fatal import error: {e}")
    sys.exit(1)

# --- 3. Performance Configuration ---
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu' 
INPUT_SIZE = 256 
DEPTH_MODEL_NAME = 'depth_anything_v2_vits' 
INTERPOLATION_MODE = 'bicubic' 

# --- 4. Model Loading ---
print(f"Loading models on {DEVICE}...")

# 4a. Load YOLOv8
yolo_model = YOLO('yolov8n.pt')
print("YOLOv8n model loaded successfully.")

# 4b. Load Depth Anything V2
try:
    print(f"Manual loading of {DEPTH_MODEL_NAME}...")
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
    }
    encoder = DEPTH_MODEL_NAME.split('_')[-1] 
    depth_model = DepthAnythingV2(**model_configs[encoder]).to(DEVICE).eval()
    
    # Check for local .pth file, otherwise download
    pth_path = f'{DEPTH_MODEL_NAME}.pth'
    if os.path.exists(pth_path):
         depth_model.load_state_dict(torch.load(pth_path, map_location=DEVICE))
    else:
         # Fallback to URL loading
         model_urls = {
            'depth_anything_v2_vits': 'https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true',
         }
         state_dict = torch.hub.load_state_dict_from_url(model_urls[DEPTH_MODEL_NAME], map_location=DEVICE)
         depth_model.load_state_dict(state_dict)

    print(f"Depth Anything V2 model ({DEPTH_MODEL_NAME}) loaded.")

except Exception as e:
    print(f"Failed to load {DEPTH_MODEL_NAME}: {e}")
    sys.exit(1)

# --- 5. Transform Definition ---
transform = Compose([ 
    Resize(
        width=INPUT_SIZE, 
        height=INPUT_SIZE, 
        resize_target=False, 
        keep_aspect_ratio=True,
        ensure_multiple_of=14,
        resize_method='lower_bound',
        image_interpolation_method=cv2.INTER_CUBIC,
    ),
    NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    PrepareForNet(), 
])

# --- 6. Camera Startup ---
CAM_ID_0 = 1 
CAM_ID_1 = 2 

cap0 = cv2.VideoCapture(CAM_ID_0) 
cap1 = cv2.VideoCapture(CAM_ID_1)

# Fallback: Try index 0 and 1 if default IDs fail
if not cap0.isOpened() or not cap1.isOpened():
    print("Attempting with index 0 and 1...")
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)

if not cap0.isOpened() or not cap1.isOpened():
    print(f"Error: Unable to open cameras.")
    sys.exit(1)

print(f"Cameras started.")

# --- Calibration Block ---
known_relative_vals = [0.10, 0.40, 0.90] 
known_real_meters   = [5.0,  3.0,  1.0]

IGNORE_CLOSE_DIST = 1.2  # Unit: meters (Do not send below this) -> Red zone
IGNORE_FAR_DIST   = 4.5  # Unit: meters (Do not send above this) -> Blue zone

# --- NETWORK INITIALIZATION ---
sock = None
connected = False

def connect_to_pi():
    global sock, connected
    print(f"Attempting connection to {RASPBERRY_IP}:{RASPBERRY_PORT}...")
    try:
        if sock: sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5) # Short timeout to prevent blocking video
        sock.connect((RASPBERRY_IP, RASPBERRY_PORT))
        print("✅ Ethernet connection successful!")
        connected = True
    except Exception as e:
        # Do not exit, continue retrying in background
        print(f"⚠️ No connection ({e}). Script continues in local mode.")
        connected = False

# First attempt
connect_to_pi()

# --- 7. Sequential Pipeline ---
try:
    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        
        if not ret0 or not ret1:
            print("Camera read error.")
            break
        
        # Merge images horizontally
        frame = np.hstack((frame0, frame1))
        H, W, _ = frame.shape
        
        # --- 7a. Depth Anything Processing ---
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
        frame_tensor = transform({'image': frame_rgb})['image'] 
        frame_tensor = torch.from_numpy(frame_tensor).unsqueeze(0).to(DEVICE)
        
        with torch.no_grad():
            depth_tensor = depth_model(frame_tensor)

        depth_tensor_resized = F.interpolate(
            depth_tensor.unsqueeze(0), size=(H, W), mode=INTERPOLATION_MODE, align_corners=None
        ).squeeze() 

        depth_resized = depth_tensor_resized.cpu().numpy()
        depth_normalized = (depth_resized - depth_resized.min()) / (depth_resized.max() - depth_resized.min() + 1e-6)
        
        # --- 7b. YOLOv8 Prediction ---
        yolo_results = yolo_model.predict(frame, verbose=False)
        
        # --- 7c. Colormap Application ---
        depth_colormap = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)

        # --- Variables to find the closest object ---
        closest_obj_data = None
        min_distance_found = 999.0

        # --- 7d. Fusion and Display (Filtered Version) ---
        for result in yolo_results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
                label = yolo_model.names[int(box.cls[0])]
                confidence = float(box.conf[0])
                
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                if 0 <= cy < H and 0 <= cx < W:
                    # Calculate distance
                    dist_rel = depth_normalized[cy, cx]
                    dist_m = np.interp(dist_rel, known_relative_vals, known_real_meters)
                    
                    # 🛑 Filter Logic 🛑
                    # If object distance is in ignore range, mark gray and skip
                    if dist_m < IGNORE_CLOSE_DIST or dist_m > IGNORE_FAR_DIST:
                        # Draw gray box (BGR: 128, 128, 128)
                        cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (128, 128, 128), 1)
                        cv2.putText(depth_colormap, f"{label} {dist_m:.1f}m (Ignored)", (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
                        continue  # Skip directly, do not update closest_obj_data
                    
                    # --- Valid Object Calculation Logic ---

                    # --- NAVIGATION CALCULATIONS ---
                    FOV = 170.0 
                    angle_deg = (cx - W/2) / (W/2) * (FOV / 2)
                    
                    lateral_offset = dist_m * math.tan(math.radians(angle_deg))
                    
                    text = f"{label} {dist_m:.2f}m"

                    # Closest object logic (Compare valid objects only)
                    if dist_m < min_distance_found:
                        min_distance_found = dist_m
                        
                        seen_by_left = 1 if cx < (W / 2) else 0
                        seen_by_right = 1 if cx >= (W / 2) else 0
                        
                        layer = 1 if dist_m < 1.5 else (2 if dist_m < 3.0 else 3)
                        
                        closest_obj_data = (
                            f"{label},"
                            f"{dist_m:.2f},"
                            f"{lateral_offset:.2f},"
                            f"{angle_deg:.1f},"
                            f"{layer},"
                            f"0.85," 
                            f"{confidence:.2f}," 
                            f"{seen_by_left}," 
                            f"{seen_by_right}" 
                        )
                    
                    # Draw valid object (White thick box)
                    cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (255, 255, 255), 2)
                    cv2.putText(depth_colormap, text, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                else:
                    # Object center out of bounds
                    text = f"{label} (Out of frame)"
                    cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (100, 100, 100), 1)

        # --- 8. Network Transmission ---
        if closest_obj_data is not None:
            csv_msg = closest_obj_data + "\n"
            
            if connected:
                try:
                    sock.sendall(csv_msg.encode('utf-8'))
                    # Print only on success to avoid console spam
                    print(f">> SENT: {csv_msg.strip()}") 
                except Exception as e:
                    print(f"CONNECTION LOST: {e}")
                    connected = False
                    sock.close()
            else:
                # Reconnection mechanism: Try to reconnect every 2 seconds
                if int(time.time()) % 2 == 0:
                    connect_to_pi()
                    
        # --- 9. Display ---
        cv2.imshow('YOLO Fusion on Depth Map', depth_colormap)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    if sock: sock.close()
    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()
    print("Program terminated.")