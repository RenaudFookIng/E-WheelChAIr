# Message pour les commandes des servomoteurs
# Contient à la fois les valeurs normalisées et les angles réels

float32 x_normalized  # Valeur X normalisée (-1.0 à 1.0)
float32 y_normalized  # Valeur Y normalisée (-1.0 à 1.0)
int32 x_angle         # Angle X réel pour le servo (0-180)
int32 y_angle         # Angle Y réel pour le servo (0-180)
float32 timestamp     # Timestamp de la commande
bool emergency_stop   # Indicateur d'arrêt d'urgence