import cv2
import torch
import threading
import queue
import time
import math
import socket
import numpy as np
import torch.nn.functional as F
from ultralytics import YOLO


# ================= YOLO BUFFER =================
class YOLOBuffer:
    def __init__(self, size):
        self.size = size
        self.history = []

    def update(self, boxes):
        self.history.append(boxes)
        if len(self.history) > self.size:
            self.history.pop(0)

    def stable(self):
        out = []
        for h in self.history:
            out.extend(h)
        return out
# ===============================================


class VisionSystem:
    def __init__(self, cfg):
        self.cfg = cfg

        self.frame_q = queue.Queue(maxsize=2)
        self.yolo_q  = queue.Queue(maxsize=2)
        self.depth_q = queue.Queue(maxsize=2)
        self.net_q   = queue.Queue(maxsize=5)

        self.yolo_model = YOLO(cfg.YOLO_MODEL)

        if cfg.DEVICE == "cpu":
            torch.set_num_threads(cfg.CPU_THREADS)

    # ================= CAMERA =================
    def camera_thread(self):
        cap0 = cv2.VideoCapture(0)
        cap1 = cv2.VideoCapture(1)

        while True:
            r0,f0 = cap0.read()
            r1,f1 = cap1.read()
            if not r0 or not r1:
                continue

            frame = np.hstack((f0,f1))
            if not self.frame_q.full():
                self.frame_q.put(frame)

    # ================= YOLO ===================
    def yolo_thread(self):
        buffer = YOLOBuffer(self.cfg.YOLO_BUFFER_SIZE)
        last = 0

        while True:
            if self.frame_q.empty():
                continue

            if time.time() - last < self.cfg.YOLO_PERIOD:
                continue

            frame = self.frame_q.queue[-1]
            last = time.time()

            results = self.yolo_model(
                frame,
                device=self.cfg.DEVICE,
                verbose=False
            )[0]

            buffer.update(results.boxes)

            if not self.yolo_q.full():
                self.yolo_q.put(buffer.stable())

    # ================= DEPTH ==================
    def depth_thread(self):
        from depth_anything_v2.dpt import DepthAnythingV2
        from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet
        from torchvision.transforms import Compose

        transform = Compose([
            Resize(self.cfg.DEPTH_INPUT, self.cfg.DEPTH_INPUT,
                   keep_aspect_ratio=True, resize_target=False,
                   ensure_multiple_of=14, resize_method="lower_bound",
                   image_interpolation_method=cv2.INTER_CUBIC),
            NormalizeImage(mean=[0.485,0.456,0.406],
                           std=[0.229,0.224,0.225]),
            PrepareForNet()
        ])

        model = DepthAnythingV2(**self.cfg.DEPTH_CFG).to(self.cfg.DEVICE).eval()
        model.load_state_dict(
            torch.load(self.cfg.DEPTH_WEIGHTS, map_location=self.cfg.DEVICE)
        )

        prev = None

        while True:
            if self.frame_q.empty():
                continue

            frame = self.frame_q.queue[-1]
            h,w,_ = frame.shape

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
            t = transform({'image': img})['image']
            t = torch.from_numpy(t).unsqueeze(0).to(self.cfg.DEVICE)

            with torch.no_grad():
                d = model(t)

            d = F.interpolate(d.unsqueeze(0), (h,w), mode="bicubic").squeeze()
            d = d.detach().cpu().numpy()
            d = (d-d.min())/(d.max()-d.min()+1e-6)

            if prev is None:
                prev = d
            d = 0.6*d + 0.4*prev
            prev = d

            if not self.depth_q.full():
                self.depth_q.put(d)

    # ================= NETWORK =================
    def network_thread(self):
        sock = None
        last_try = 0

        while True:
            if self.net_q.empty():
                continue

            msg = self.net_q.get()

            if sock is None and time.time()-last_try > 2:
                last_try = time.time()
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(0.2)
                    sock.connect((self.cfg.RASPBERRY_IP, self.cfg.RASPBERRY_PORT))
                except:
                    sock = None

            if sock:
                try:
                    sock.sendall((msg+"\n").encode())
                except:
                    sock = None

    # ================= MAIN LOOP ===============
    def run(self):
        threading.Thread(target=self.camera_thread, daemon=True).start()
        threading.Thread(target=self.yolo_thread, daemon=True).start()
        threading.Thread(target=self.depth_thread, daemon=True).start()
        threading.Thread(target=self.network_thread, daemon=True).start()

        while True:
            if self.frame_q.empty() or self.depth_q.empty() or self.yolo_q.empty():
                continue

            frame = self.frame_q.get()
            depth = self.depth_q.get()
            boxes = self.yolo_q.get()

            H,W,_ = frame.shape
            min_d = 99
            msg = None

            for b in boxes:
                x1,y1,x2,y2 = map(int,b.xyxy[0])
                cx,cy = (x1+x2)//2,(y1+y2)//2
                if cx<0 or cy<0 or cx>=W or cy>=H:
                    continue

                d = np.interp(depth[cy,cx],[0.1,0.4,0.9],[5,3,1])
                zone = self.cfg.zone_from_distance(d)

                cv2.rectangle(frame,(x1,y1),(x2,y2),self.cfg.ZONE_COLORS[zone],2)
                cv2.putText(frame,f"{zone} {d:.2f}m",(x1,y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,
                            self.cfg.ZONE_COLORS[zone],2)

                if d < min_d:
                    min_d = d
                    angle = (cx-W/2)/(W/2)*(170/2)
                    lat = d * math.tan(math.radians(angle))
                    layer = 1 if d<1.5 else 2 if d<3 else 3

                    msg = (
                        f"{int(b.cls[0])},{d:.2f},{lat:.2f},{angle:.1f},"
                        f"{layer},0.85,{float(b.conf[0]):.2f},"
                        f"{1 if cx<W/2 else 0},{1 if cx>=W/2 else 0}"
                    )

            if msg:
                self.net_q.put(msg)

            cv2.imshow(self.cfg.WINDOW_NAME, frame)
            if cv2.waitKey(1) == 27:
                break
