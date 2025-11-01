import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/renaud/Blade 15 HDD/M2_Cours/Designing/sabertooth_ws/src/sabertooth_controller/install/sabertooth_controller'
