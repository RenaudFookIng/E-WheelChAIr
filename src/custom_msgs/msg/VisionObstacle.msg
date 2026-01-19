# ================================
#   Vision Obstacle (fusion PC)
# ================================

# Identification
string obstacle_type        # "person", "wall", "chair", "unknown"

# Position relative par rapport à l’axe du fauteuil
float32 angle_deg           # [-90 ; +90] (0 = droit devant)
float32 lateral_offset      # [-1 ; +1] gauche(-) / droite(+)
float32 forward_offset      # [0 ; 1] avant

# Profondeur (Depth Anything)
uint8 depth_layer           # 1 = proche, 2 = moyen, 3 = loin
float32 depth_confidence    # confiance estimation profondeur

# Détection (YOLO + fusion)
float32 detection_confidence  # [0 ; 1]

# Caméras impliquées
bool seen_by_left_camera
bool seen_by_right_camera

# Métadonnées
builtin_interfaces/Time stamp
