import sys
import os

# ---------------- Ajouter Depth-Anything-V2 au PYTHONPATH ----------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DA_PATH = os.path.join(SCRIPT_DIR, "Depth-Anything-V2")  # chemin relatif vers le module
if DA_PATH not in sys.path:
    sys.path.append(DA_PATH)

# ---------------- Imports standards ----------------
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
    """Permet de garder un historique des boxes YOLO pour stabiliser la détection"""
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

        # ================= Queues =================
        self.frame_q = queue.Queue(maxsize=2)
        self.yolo_q  = queue.Queue(maxsize=2)
        self.depth_q = queue.Queue(maxsize=2)
        self.net_q   = queue.Queue(maxsize=5)

        # ================= Modèles =================
        self.yolo_model = YOLO(cfg.YOLO_MODEL)
        if cfg.DEVICE == "cpu":
            torch.set_num_threads(cfg.CPU_THREADS)

        # Compteurs pour traiter seulement certaines frames
        self.frame_count_yolo = 0
        self.frame_count_depth = 0

    def merge_frames(self, frame0, frame1, target_height=720):
        # Resize pour avoir la même hauteur
        frame0 = cv2.resize(frame0, (int(frame0.shape[1]*target_height/frame0.shape[0]), target_height))
        frame1 = cv2.resize(frame1, (int(frame1.shape[1]*target_height/frame1.shape[0]), target_height))

        # Calcul crop pour éviter chevauchement
        crop0 = int(frame0.shape[1] * self.cfg.CROP_PERCENT)
        crop1 = int(frame1.shape[1] * self.cfg.CROP_PERCENT)

        frame0_cropped = frame0[:, crop0:-crop0]
        frame1_cropped = frame1[:, crop1:-crop1]

        merged = np.hstack((frame0_cropped, frame1_cropped))
        return merged


    # ================= CAMERA =================
    def camera_thread(self):
        cap0 = cv2.VideoCapture(1) # Test avec deux caméras USB trouver le bon ID
        cap1 = cv2.VideoCapture(0) # Test avec deux caméras USB trouver le bon ID

        while True:
            r0,f0 = cap0.read()
            r1,f1 = cap1.read()
            if not r0 or not r1:
                continue

            frame = self.merge_frames(f0, f1, target_height=self.cfg.TARGET_HEIGHT)
            # Toujours mettre la dernière frame disponible
            if not self.frame_q.full():
                self.frame_q.put(frame)
            else:
                try:
                    self.frame_q.get_nowait()
                except:
                    pass
                self.frame_q.put(frame)

    # ================= YOLO ===================
    def yolo_thread(self):
        """Détecte les objets avec YOLO, mais seulement une frame sur 2 pour économiser CPU"""
        buffer = YOLOBuffer(self.cfg.YOLO_BUFFER_SIZE)
        last = 0

        while True:
            try:
                frame = self.frame_q.get(timeout=0.01)
            except queue.Empty:
                continue

            self.frame_count_yolo += 1
            if self.frame_count_yolo % self.cfg.FRAME_SKIP_YOLO != 0:  # Ne traiter qu'une frame sur 2X defini dans vision_config_mac.py
                continue

            #frame = self.frame_q.queue[-1]
            last = time.time()

            # ------------------ Mesure temps YOLO ------------------
            #start = time.time()
            results = self.yolo_model(frame, device=self.cfg.DEVICE, verbose=False)[0]
            #yolo_time = time.time() - start
            #print(f"[YOLO] frame {self.frame_count_yolo} processed in {yolo_time:.3f}s")
            # --------------------------------------------------------

            buffer.update(results.boxes)

            if not self.yolo_q.full():
                self.yolo_q.put(buffer.stable())

    # ================= DEPTH ==================
    def depth_thread(self):
        """Calcule la carte de profondeur, mais seulement une frame sur 2"""
        from depth_anything_v2.dpt import DepthAnythingV2
        from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet
        from torchvision.transforms import Compose
        import torch.hub

        # Transformation adaptée pour Mac CPU
        transform = Compose([
            Resize(self.cfg.DEPTH_INPUT, self.cfg.DEPTH_INPUT,
                   keep_aspect_ratio=True, resize_target=False,
                   ensure_multiple_of=14, resize_method="lower_bound",
                   image_interpolation_method=cv2.INTER_CUBIC),
            NormalizeImage(mean=[0.485,0.456,0.406],
                           std=[0.229,0.224,0.225]),
            PrepareForNet()
        ])

        # ----------------- Initialisation du modèle -----------------
        # Si absent chargement depuis URL
        # -------------------------------------------------------------
        model = DepthAnythingV2(**self.cfg.DEPTH_CFG).to(self.cfg.DEVICE).eval()

        # Téléchargement automatique du fichier poids s'il n'existe pas
        if os.path.exists(self.cfg.DEPTH_WEIGHTS):
            print(f"Loading local weights: {self.cfg.DEPTH_WEIGHTS}")
            model.load_state_dict(torch.load(self.cfg.DEPTH_WEIGHTS, map_location=self.cfg.DEVICE))
        else:
            print(f"Local weights not found, downloading...")
            urls = {
                'depth_anything_v2_vits': 'https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true',
                'depth_anything_v2_vitb': 'https://huggingface.co/depth-anything/Depth-Anything-V2-Medium/resolve/main/depth_anything_v2_vitb.pth?download=true',
                'depth_anything_v2_vitl': 'https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth?download=true',
            }
            model_name = os.path.basename(self.cfg.DEPTH_WEIGHTS).replace('.pth','')
            if model_name not in urls:
                raise ValueError(f"No URL configured for {model_name}")
            state_dict = torch.hub.load_state_dict_from_url(urls[model_name], map_location=self.cfg.DEVICE)

            # Sauvegarde localement pour réutilisation
            os.makedirs(os.path.dirname(self.cfg.DEPTH_WEIGHTS), exist_ok=True)
            torch.save(state_dict, self.cfg.DEPTH_WEIGHTS)
            print(f"Weights saved locally at {self.cfg.DEPTH_WEIGHTS}")

            model.load_state_dict(state_dict)
            print("Download complete.")

        prev = None

        while True:
            try:
                frame = self.frame_q.get(timeout=0.01)
            except queue.Empty:
                continue

            self.frame_count_depth += 1
            if self.frame_count_depth % self.cfg.FRAME_SKIP_DEPTH != 0:  # Ne traiter qu'une frame sur valeur définie dans vision_config_mac.py
                continue

            h,w,_ = frame.shape

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
            t = transform({'image': img})['image']
            t = torch.from_numpy(t).unsqueeze(0).to(self.cfg.DEVICE)

            # ------------------ Mesure temps Depth ------------------
            #start = time.time()
            with torch.no_grad():
                d = model(t)
            #depth_time = time.time() - start
            #print(f"[Depth] frame {self.frame_count_depth} processed in {depth_time:.3f}s")
            # ---------------------------------------------------------

            d = F.interpolate(d.unsqueeze(0), (h,w), mode="bicubic").squeeze().cpu().numpy()
            d = (d-d.min())/(d.max()-d.min()+1e-6)

            if prev is None:
                prev = d
            d = 0.6*d + 0.4*prev
            prev = d

            if not self.depth_q.full():
                self.depth_q.put(d)

    def zone_from_relative_depth(self, d_rel):
        """
        d_rel : float ∈ [0,1], 1 = très proche, 0 = très loin
        Retourne un nom de zone et un code couleur
        """
        if d_rel > 0.75:
            return "ROUGE", (0,0,255)       # danger
        elif d_rel > 0.5:
            return "ORANGE", (0,165,255)   # attention rapide
        elif d_rel > 0.25:
            return "JAUNE", (0,255,255)    # évitement normal
        else:
            return "VERT", (0,255,0)       # safe


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
        """Lance tous les threads"""
        threading.Thread(target=self.camera_thread, daemon=True).start()
        threading.Thread(target=self.yolo_thread, daemon=True).start()
        threading.Thread(target=self.depth_thread, daemon=True).start()
        threading.Thread(target=self.network_thread, daemon=True).start()

        while True:
            # Lecture sécurisée des queues
            try:
                frame = self.frame_q.get(timeout=0.01)
                depth = self.depth_q.get(timeout=0.01)
                boxes = self.yolo_q.get(timeout=0.01)
            except queue.Empty:
                continue

            H,W,_ = frame.shape
            min_d = 99 #-1
            msg = None

            # --- Traitement des boxes YOLO ---
            for b in boxes:
                x1,y1,x2,y2 = map(int,b.xyxy[0])
                cx,cy = (x1+x2)//2,(y1+y2)//2
                if cx<0 or cy<0 or cx>=W or cy>=H:
                    continue
                
                # Distance estimée avec depth
                #d = np.interp(depth[cy,cx],[0.1,0.4,0.9],[5,3,1])
                #zone = self.cfg.zone_from_distance(d)

                # Profondeur relative [0,1]
                d_rel = depth[cy, cx]
                # Déterminer zone + couleur
                zone_name, color = self.zone_from_relative_depth(d_rel)

                # Label YOLO lisible
                yolo_label = self.yolo_model.names[int(b.cls[0])]
                # Filtrage selon ta règle :
                # - Afficher toujours si humain
                # - Sinon afficher uniquement si zone jaune, orange ou rouge
                if yolo_label != "person" and zone_name == "VERT":
                    continue  # ignorer cet objet

                label_text = f"{yolo_label} {d_rel:.2f} {zone_name}"


                # Affichage (optionnel pour debug)
                if self.cfg.SHOW_DISPLAY:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label_text, (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    
                # Calcul angle et latéral pour pilotage
                angle = (cx - W/2) / (W/2) * (170/2)  # angle approximatif
                lat = d_rel * math.tan(math.radians(angle))

                # Layer pour le réseau : 1=ROUGE, 2=ORANGE, 3=JAUNE, 4=VERT
                layer = 1 if zone_name == "ROUGE" else 2 if zone_name == "ORANGE" else 3 if zone_name == "JAUNE" else 4

                # Message réseau pour l'objet le plus proche
                if min_d < 0 or d_rel > min_d:  # Depth Anything : valeur proche = plus grande
                    min_d = d_rel
                    #angle = (cx-W/2)/(W/2)*(170/2)
                    #lat = d_rel * math.tan(math.radians(angle))
                    
                    msg = (f"{int(b.cls[0])},"
                        f"{d_rel:.3f},"
                        f"{lat:.2f},"
                        f"{angle:.1f},"
                        f"{layer},"
                        f"0.85,"
                        f"{float(b.conf[0]):.2f},"
                        f"{1 if cx<W/2 else 0},"
                        f"{1 if cx>=W/2 else 0}")
                                        
            # Envoyer message réseau
            if msg:
                self.net_q.put(msg)

            # --- AFFICHAGE dans le main thread (Mac safe) ---
            # Affichage headless / debug
            if self.cfg.SHOW_DISPLAY:
                cv2.imshow(self.cfg.WINDOW_NAME, frame)
                if cv2.waitKey(1) == 27:  # ESC pour quitter
                    break

        # --- Nettoyage ---
        if self.cfg.SHOW_DISPLAY:
            cv2.destroyAllWindows()
