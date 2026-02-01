class VisionConfigMac:
    DEVICE = "cpu"
    CPU_THREADS = 4

    YOLO_MODEL = "yolov8n.pt"
    YOLO_BUFFER_SIZE = 5
    YOLO_PERIOD = 0.18

    DEPTH_INPUT = 256
    DEPTH_WEIGHTS = "depth_anything_v2_vits.pth"
    DEPTH_CFG = dict(encoder='vits', features=64, out_channels=[48,96,192,384])

    RASPBERRY_IP = "192.168.50.2"
    RASPBERRY_PORT = 5000

    WINDOW_NAME = "Vision MAC"

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
