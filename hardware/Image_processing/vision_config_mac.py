import os

class VisionConfigMac:
    DEVICE = "cpu"
    CPU_THREADS = 4

    YOLO_MODEL = "yolov8n.pt"
    YOLO_BUFFER_SIZE = 5
    YOLO_PERIOD = 0.18

    DEPTH_INPUT = 256

    CROP_PERCENT = 0.10  # % des côtés à retirer pour éviter doublons
    TARGET_FPS = 30
    TARGET_HEIGHT = 480
    FRAME_SKIP_YOLO = 10
    FRAME_SKIP_DEPTH = 10

    # CHEMIN COMPLET DU .PTH
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    DEPTH_WEIGHTS = os.path.join(SCRIPT_DIR, "Depth-Anything-V2", "depth_anything_v2_vits.pth")

    DEPTH_CFG = dict(encoder='vits', features=64, out_channels=[48,96,192,384])

    RASPBERRY_IP = "192.168.50.2"
    RASPBERRY_PORT = 5000

    WINDOW_NAME = "Vision MAC"
    
    SHOW_DISPLAY = True #False pour eviter l'affichage

    ZONE_COLORS = {
        "SAFE":  (0,255,0),
        "WATCH": (0,255,255),
        "AVOID": (0,165,255),
        "STOP":  (0,0,255)
    }

    @staticmethod
    def zone_from_distance(d):
        if d > 4: return "SAFE"
        if d > 2.5: return "WATCH"
        if d > 1.5: return "AVOID"
        return "STOP"
