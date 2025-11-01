import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/renaud/Blade 15 HDD/M2_Cours/Designing/E-WheelChAIr/EWheelChAIr_ws/install/yolo_detector'
