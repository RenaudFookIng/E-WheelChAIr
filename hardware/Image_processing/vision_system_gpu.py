from hardware.Image_processing.vision_core_depth import VisionSystem
from vision_config_gpu import VisionConfigGPU

if __name__ == "__main__":
    VisionSystem(VisionConfigGPU()).run()
