#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from ultralytics import YOLO


# -------------------- Utils --------------------
def center_dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def stamp_to_sec(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def safe_norm(v, eps=1e-12):
    n = float(np.linalg.norm(v))
    return v / (n + eps)


def cosine_sim(a, b):
    return float(np.dot(a, b))


def iou_xyxy(b1, b2):
    x1, y1, x2, y2 = b1
    X1, Y1, X2, Y2 = b2
    ix1, iy1 = max(x1, X1), max(y1, Y1)
    ix2, iy2 = min(x2, X2), min(y2, Y2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    a1 = max(0, x2 - x1) * max(0, y2 - y1)
    a2 = max(0, X2 - X1) * max(0, Y2 - Y1)
    den = a1 + a2 - inter
    return inter / den if den > 1e-6 else 0.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# -------------------- Appearance extractor (ReID if available, else HSV) --------------------
class AppearanceExtractor:
    """
    Returns a normalized feature vector (np.float32).
    Uses OSNet torchreid if available, else HSV histogram fallback.
    """
    def __init__(self, prefer_reid=True):
        self.mode = "HSV"
        self.torch = None
        self.reid = None
        self.device = None
        self.img_h, self.img_w = 256, 128

        if prefer_reid:
            try:
                import torch  # noqa
                import torchreid  # noqa
                from torchvision import transforms  # noqa

                self.torch = torch
                self.transforms = transforms.Compose([
                    transforms.ToPILImage(),
                    transforms.Resize((self.img_h, self.img_w)),
                    transforms.ToTensor(),
                    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225]),
                ])

                self.device = "cuda" if torch.cuda.is_available() else "cpu"
                self.reid = torchreid.models.build_model(
                    name="osnet_x1_0",
                    num_classes=1000,
                    pretrained=True
                )
                self.reid.to(self.device)
                self.reid.eval()
                self.mode = "REID_OSNET"
            except Exception:
                self.mode = "HSV"

    def extract(self, crop_bgr: np.ndarray) -> np.ndarray | None:
        if crop_bgr is None or crop_bgr.size == 0:
            return None
        h, w = crop_bgr.shape[:2]
        if h < 20 or w < 20:
            return None

        if self.mode == "REID_OSNET":
            try:
                torch = self.torch
                rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
                x = self.transforms(rgb).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    feat = self.reid(x)
                feat = feat.detach().cpu().numpy().astype(np.float32).reshape(-1)
                return safe_norm(feat)
            except Exception:
                self.mode = "HSV"

        hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 8, 8],
                            [0, 180, 0, 256, 0, 256])
        hist = hist.astype(np.float32).flatten()
        return safe_norm(hist)


# -------------------- Node --------------------
class PersonDetectorYOLONode(Node):
    """
    Patch temps réel inclus:
    - QoS BEST_EFFORT + KEEP_LAST(depth=1) => évite l'accumulation (backlog)
    - on stocke seulement la DERNIÈRE image reçue, et on traite via un TIMER à fréquence fixée
    - resize avant YOLO (yolo_scale) + remap bbox => gros gain perf
    """

    def __init__(self):
        super().__init__('person_detector_yolo_node')

        self.bridge = CvBridge()

        # -------------------- Params --------------------
        self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
        self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("click_target_tol_px", 60)
        self.declare_parameter("depth_roi_k", 5)

        # -------- real-time patch params --------
        self.declare_parameter("process_rate_hz", 10.0)   # fréquence de traitement YOLO (ex: 10 Hz)
        self.declare_parameter("yolo_scale", 0.5)         # resize avant YOLO (0.5=div2)
        self.declare_parameter("publish_image", True)

        # -------- robust ID memory params --------
        self.declare_parameter("max_lost_sec", 60.0)
        self.declare_parameter("reacquire_top_k", 3)
        self.declare_parameter("reacquire_gate_px", 700)
        self.declare_parameter("depth_gate_m", 0.60)
        self.declare_parameter("area_gate_ratio", 0.60)
        self.declare_parameter("min_cosine", 0.75)
        self.declare_parameter("prefer_reid", True)
        self.declare_parameter("appearance_update_every_n", 3)
        self.declare_parameter("ema_alpha", 0.15)
        self.declare_parameter("use_iou_bonus", True)

        # ------- read params -------
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.fx = float(self.get_parameter("fx").value)
        self.click_target_tol_px = int(self.get_parameter("click_target_tol_px").value)
        self.depth_roi_k = int(self.get_parameter("depth_roi_k").value)

        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self.yolo_scale = float(self.get_parameter("yolo_scale").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)

        self.max_lost_sec = float(self.get_parameter("max_lost_sec").value)
        self.reacquire_top_k = int(self.get_parameter("reacquire_top_k").value)
        self.reacquire_gate_px = int(self.get_parameter("reacquire_gate_px").value)
        self.depth_gate_m = float(self.get_parameter("depth_gate_m").value)
        self.area_gate_ratio = float(self.get_parameter("area_gate_ratio").value)
        self.min_cosine = float(self.get_parameter("min_cosine").value)
        self.prefer_reid = bool(self.get_parameter("prefer_reid").value)
        self.appearance_update_every_n = int(self.get_parameter("appearance_update_every_n").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.use_iou_bonus = bool(self.get_parameter("use_iou_bonus").value)

        # -------------------- YOLO --------------------
        model_path = self.get_parameter("yolo_model").value
        self.get_logger().info(f'Chargement du modèle YOLO ({model_path})...')
        self.model = YOLO(model_path)
        self.get_logger().info('Modèle YOLO chargé.')
        self.person_class_id = 0

        # -------------------- QoS (IMPORTANT) --------------------
        # depth=1 + best effort => pas d'accumulation d'images
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_rx_callback, qos)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, qos)

        # -------------------- Publishers --------------------
        self.rgb_pub = self.create_publisher(Image, 'image_box', 10)
        self.polar_pub = self.create_publisher(PointStamped, 'person_polar', 10)

        # -------------------- Incoming buffers (latest only) --------------------
        self.latest_rgb_msg = None
        self.latest_rgb_cv = None
        self.latest_rgb_stamp = None

        self.depth_image = None

        # -------------------- Detections for mouse --------------------
        self.last_detections = []

        # -------------------- State machine --------------------
        self.track_state = "IDLE"  # IDLE / LOCKED / LOST
        self.lost_since = None

        self.locked_center = None
        self.locked_bbox = None
        self.locked_distance = None
        self.locked_area = None

        self.prev_locked_center = None
        self.last_lock_stamp = None
        self.lock_velocity = (0.0, 0.0)

        # appearance memory
        self.appearance = AppearanceExtractor(prefer_reid=self.prefer_reid)
        self.locked_feat = None
        self.gallery = deque(maxlen=30)
        self.frame_count = 0

        self.get_logger().info(f"Appearance mode: {self.appearance.mode}")

        # -------------------- OpenCV --------------------
        self.window_name = "Person Selector"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        # -------------------- Timer processing loop --------------------
        period = 1.0 / max(1e-3, self.process_rate_hz)
        self.timer = self.create_timer(period, self.process_latest_frame)

        self.get_logger().info(
            "Node démarré.\n"
            f"  RGB  : {self.rgb_topic}\n"
            f"  DEPTH: {self.depth_topic}\n"
            f"  process_rate_hz: {self.process_rate_hz}\n"
            f"  yolo_scale: {self.yolo_scale}\n"
            "  OUT IMG  : /image_box\n"
            "  OUT POLAR: /person_polar\n"
        )

    # ---------- RX RGB: store latest only ----------
    def rgb_rx_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Erreur CvBridge (rgb): {e}')
            return

        self.latest_rgb_msg = msg
        self.latest_rgb_cv = cv_image
        self.latest_rgb_stamp = msg.header.stamp

    # ---------- Depth callback ----------
    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Erreur CvBridge (depth): {e}')
            return
        self.depth_image = depth

    # ---------- Distance median ROI ----------
    def get_distance_m(self, cx: int, cy: int, k: int = 5):
        if self.depth_image is None:
            return None
        try:
            h, w = self.depth_image.shape[:2]
            if not (0 <= cx < w and 0 <= cy < h):
                return None

            x1 = max(0, cx - k)
            x2 = min(w - 1, cx + k)
            y1 = max(0, cy - k)
            y2 = min(h - 1, cy + k)

            roi = self.depth_image[y1:y2 + 1, x1:x2 + 1].astype(np.float32)
            roi = roi[~np.isnan(roi)]
            roi = roi[roi > 0.0]
            if roi.size == 0:
                return None

            med = float(np.median(roi))
            if self.depth_image.dtype == np.uint16:
                return med / 1000.0
            return med
        except Exception as e:
            self.get_logger().warn(f'Erreur lecture profondeur: {e}')
            return None

    # ---------- Crop ----------
    def crop_det(self, frame_bgr, det, pad=8):
        h, w = frame_bgr.shape[:2]
        x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
        x1 = max(0, x1 - pad)
        y1 = max(0, y1 - pad)
        x2 = min(w - 1, x2 + pad)
        y2 = min(h - 1, y2 + pad)
        if x2 <= x1 or y2 <= y1:
            return None
        return frame_bgr[y1:y2, x1:x2].copy()

    # ---------- Predict center ----------
    def predict_center(self, now_t):
        if self.locked_center is None or self.last_lock_stamp is None:
            return self.locked_center
        dt = max(1e-3, now_t - self.last_lock_stamp)
        vx, vy = self.lock_velocity
        px = int(self.locked_center[0] + vx * dt)
        py = int(self.locked_center[1] + vy * dt)
        return (px, py)

    # ---------- Candidate gating ----------
    def gated_candidates(self, detections, pred_center):
        if not detections:
            return []
        cand = []
        px, py = pred_center if pred_center is not None else (None, None)
        gate2 = self.reacquire_gate_px * self.reacquire_gate_px

        for i, d in enumerate(detections):
            if px is not None:
                if center_dist2((d["cx"], d["cy"]), (px, py)) > gate2:
                    continue

            if self.locked_distance is not None and d["distance"] is not None:
                if abs(d["distance"] - self.locked_distance) > self.depth_gate_m:
                    continue

            area = (d["x2"] - d["x1"]) * (d["y2"] - d["y1"])
            if self.locked_area is not None and self.locked_area > 0:
                ratio = area / float(self.locked_area)
                if abs(ratio - 1.0) > self.area_gate_ratio:
                    continue

            d2 = center_dist2((d["cx"], d["cy"]), (px, py)) if px is not None else 0
            cand.append((d2, i))

        cand.sort(key=lambda x: x[0])
        return [i for _, i in cand]

    def best_gallery_cos(self, feat):
        if feat is None or len(self.gallery) == 0:
            return -1.0
        best = -1.0
        for g in self.gallery:
            best = max(best, cosine_sim(feat, g))
        return best

    # ---------- Reacquire ----------
    def try_reacquire(self, frame_bgr, detections, pred_center):
        idxs = self.gated_candidates(detections, pred_center)
        if not idxs:
            return None, None
        idxs = idxs[:max(1, self.reacquire_top_k)]

        best_idx, best_score, best_cos = None, None, None
        px, py = pred_center if pred_center is not None else (None, None)

        for i in idxs:
            d = detections[i]
            crop = self.crop_det(frame_bgr, d)
            feat = self.appearance.extract(crop)
            if feat is None:
                continue

            cosv = self.best_gallery_cos(feat)
            if cosv < 0:
                continue

            score = 2.0 * cosv
            if self.use_iou_bonus and self.locked_bbox is not None:
                score += 0.4 * iou_xyxy((d["x1"], d["y1"], d["x2"], d["y2"]), self.locked_bbox)

            if self.locked_distance is not None and d["distance"] is not None:
                dd = abs(d["distance"] - self.locked_distance) / max(0.3, self.depth_gate_m)
                score -= 0.2 * clamp(dd, 0.0, 2.0)

            area = (d["x2"] - d["x1"]) * (d["y2"] - d["y1"])
            if self.locked_area is not None and self.locked_area > 0:
                ar = abs(area / float(self.locked_area) - 1.0)
                score -= 0.2 * clamp(ar / max(0.1, self.area_gate_ratio), 0.0, 2.0)

            if px is not None:
                dpx = math.sqrt(center_dist2((d["cx"], d["cy"]), (px, py))) / max(50.0, float(self.reacquire_gate_px))
                score -= 0.1 * clamp(dpx, 0.0, 2.0)

            if best_score is None or score > best_score:
                best_score, best_idx, best_cos = score, i, cosv

        if best_idx is None:
            return None, None
        if best_cos is not None and best_cos >= self.min_cosine:
            return best_idx, best_cos
        return None, None

    # ---------- Update appearance ----------
    def update_locked_appearance(self, frame_bgr, det):
        if self.appearance_update_every_n > 1:
            if (self.frame_count % self.appearance_update_every_n) != 0:
                return

        crop = self.crop_det(frame_bgr, det)
        feat = self.appearance.extract(crop)
        if feat is None:
            return

        self.gallery.append(feat)
        if self.locked_feat is None:
            self.locked_feat = feat
        else:
            a = clamp(self.ema_alpha, 0.01, 0.9)
            self.locked_feat = safe_norm((1.0 - a) * self.locked_feat + a * feat)

    # ---------- Main processing loop (timer) ----------
    def process_latest_frame(self):
        if self.latest_rgb_cv is None or self.latest_rgb_msg is None or self.latest_rgb_stamp is None:
            return

        self.frame_count += 1
        msg = self.latest_rgb_msg
        cv_image = self.latest_rgb_cv
        now_t = stamp_to_sec(self.latest_rgb_stamp)

        # ---- Resize BEFORE YOLO ----
        scale = float(self.yolo_scale)
        scale = clamp(scale, 0.2, 1.0)

        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if scale < 0.999:
            h, w = img_rgb.shape[:2]
            img_small = cv2.resize(img_rgb, (int(w * scale), int(h * scale)))
        else:
            img_small = img_rgb

        # ---- YOLO on resized image ----
        results = self.model(img_small, verbose=False)
        boxes = results[0].boxes

        annotated = cv_image.copy()
        current_detections = []

        # Parse detections + remap coords back to original
        for box in boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            if cls_id != self.person_class_id or conf < self.conf_threshold:
                continue

            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            if scale < 0.999:
                x1, y1, x2, y2 = x1 / scale, y1 / scale, x2 / scale, y2 / scale

            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            distance_m = self.get_distance_m(cx, cy, k=self.depth_roi_k)

            current_detections.append({
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "cx": cx, "cy": cy,
                "conf": conf,
                "distance": distance_m,
            })

        self.last_detections = current_detections

        # ------------- Decide locked_idx -------------
        locked_idx = None

        if self.track_state == "LOCKED" and self.locked_center is not None:
            tx, ty = self.locked_center
            best_i, best_d2 = None, None
            for i, d in enumerate(current_detections):
                d2 = center_dist2((d["cx"], d["cy"]), (tx, ty))
                if best_d2 is None or d2 < best_d2:
                    best_d2, best_i = d2, i

            # seuil serré uniquement en LOCKED
            if best_i is not None and best_d2 is not None and best_d2 <= (80 ** 2):
                locked_idx = best_i
            else:
                self.track_state = "LOST"
                self.lost_since = now_t
                self.get_logger().info("TARGET lost -> LOST (keep memory)")

        if self.track_state == "LOST":
            pred_center = self.predict_center(now_t)
            reacq_idx, reacq_cos = self.try_reacquire(cv_image, current_detections, pred_center)
            if reacq_idx is not None:
                locked_idx = reacq_idx
                self.track_state = "LOCKED"
                self.lost_since = None
                self.get_logger().info(f"TARGET reacquired (cos={reacq_cos:.2f})")

            if self.lost_since is not None and (now_t - self.lost_since) > self.max_lost_sec:
                self.get_logger().info("TARGET lost too long -> DELOCK")
                self.track_state = "IDLE"
                self.lost_since = None
                self.locked_center = None
                self.locked_bbox = None
                self.locked_distance = None
                self.locked_area = None
                self.prev_locked_center = None
                self.last_lock_stamp = None
                self.lock_velocity = (0.0, 0.0)
                self.locked_feat = None
                self.gallery.clear()

        # ------------- Draw detections -------------
        for idx, det in enumerate(current_detections):
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
            cx, cy = det["cx"], det["cy"]
            conf = det["conf"]
            distance_m = det["distance"]

            is_locked = (locked_idx is not None and idx == locked_idx and self.track_state == "LOCKED")
            color = (255, 0, 0) if is_locked else (0, 255, 0)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

            if is_locked:
                # motion update
                if self.last_lock_stamp is not None and self.prev_locked_center is not None:
                    dt = max(1e-3, now_t - self.last_lock_stamp)
                    self.lock_velocity = ((cx - self.prev_locked_center[0]) / dt,
                                          (cy - self.prev_locked_center[1]) / dt)
                self.prev_locked_center = (cx, cy)
                self.last_lock_stamp = now_t

                self.locked_center = (cx, cy)
                self.locked_bbox = (x1, y1, x2, y2)
                self.locked_distance = distance_m
                self.locked_area = max(1, (x2 - x1) * (y2 - y1))

                self.update_locked_appearance(cv_image, det)

                label = f"TARGET {conf:.2f}, {distance_m:.2f} m" if distance_m is not None else f"TARGET {conf:.2f}, ? m"
            else:
                label = f"person {conf:.2f}"

            cv2.putText(annotated, label, (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # ------------- LOST overlay -------------
        if self.track_state == "LOST" and self.locked_center is not None:
            pred = self.predict_center(now_t)
            if pred is not None:
                cv2.circle(annotated, pred, 6, (0, 165, 255), -1)
            txt = "LOST: searching..."
            if self.lost_since is not None:
                txt += f" ({now_t - self.lost_since:.1f}s)"
            cv2.putText(annotated, txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2, cv2.LINE_AA)

        # ------------- Publish polar for locked only -------------
        if locked_idx is not None and self.track_state == "LOCKED":
            det = current_detections[locked_idx]
            cx = det["cx"]
            distance_m = det["distance"]
            if distance_m is not None:
                h, w = annotated.shape[:2]
                cx0 = w / 2.0
                angle_rad = math.atan((float(cx) - float(cx0)) / float(self.fx))
                out = PointStamped()
                out.header = msg.header
                out.point.x = float(distance_m)
                out.point.y = float(angle_rad)
                out.point.z = 0.0
                self.polar_pub.publish(out)

        # OpenCV window
        try:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)
        except cv2.error:
            pass

        # Publish annotated image
        if self.publish_image:
            try:
                out_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out_img.header = msg.header
                self.rgb_pub.publish(out_img)
            except CvBridgeError as e:
                self.get_logger().error(f'Erreur CvBridge (pub rgb): {e}')

    # --------- Mouse callback ---------
    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        clicked_idx = None
        for idx, det in enumerate(self.last_detections):
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
            if x1 <= x <= x2 and y1 <= y <= y2:
                clicked_idx = idx
                break

        # click outside => hard delock
        if clicked_idx is None:
            if self.track_state != "IDLE":
                self.get_logger().info("DELOCK (click outside)")
            self.track_state = "IDLE"
            self.lost_since = None
            self.locked_center = None
            self.locked_bbox = None
            self.locked_distance = None
            self.locked_area = None
            self.prev_locked_center = None
            self.last_lock_stamp = None
            self.lock_velocity = (0.0, 0.0)
            self.locked_feat = None
            self.gallery.clear()
            return

        clicked = self.last_detections[clicked_idx]
        ccx, ccy = clicked["cx"], clicked["cy"]

        # click on target => delock
        if self.locked_center is not None:
            tx, ty = self.locked_center
            if center_dist2((ccx, ccy), (tx, ty)) < (self.click_target_tol_px ** 2):
                self.get_logger().info("DELOCK (clicked target)")
                self.track_state = "IDLE"
                self.lost_since = None
                self.locked_center = None
                self.locked_bbox = None
                self.locked_distance = None
                self.locked_area = None
                self.prev_locked_center = None
                self.last_lock_stamp = None
                self.lock_velocity = (0.0, 0.0)
                self.locked_feat = None
                self.gallery.clear()
                return

        # lock / switch
        self.track_state = "LOCKED"
        self.lost_since = None
        self.locked_center = (ccx, ccy)
        self.locked_bbox = (clicked["x1"], clicked["y1"], clicked["x2"], clicked["y2"])
        self.locked_distance = clicked["distance"]
        self.locked_area = max(1, (clicked["x2"] - clicked["x1"]) * (clicked["y2"] - clicked["y1"]))

        self.prev_locked_center = None
        self.last_lock_stamp = None
        self.lock_velocity = (0.0, 0.0)

        self.locked_feat = None
        self.gallery.clear()

        if self.locked_distance is not None:
            self.get_logger().info(f"LOCK (switch) - distance ≈ {self.locked_distance:.2f} m")
        else:
            self.get_logger().info("LOCK (switch) - distance inconnue")


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

