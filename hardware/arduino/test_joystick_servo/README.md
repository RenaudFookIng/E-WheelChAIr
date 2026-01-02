# E-WheelChAIr - Test Joystick & Servo Controller

## Description

Ce script Arduino simplifié est conçu pour tester le branchement et le fonctionnement du joystick et des servomoteurs avec un Arduino Uno, en préparation pour l'intégration avec ROS2 sur Raspberry Pi 3.

## Caractéristiques principales

- **Contrôle du joystick PS2** : Lecture des entrées analogiques pour les mouvements
- **Contrôle des servomoteurs MG996R** : Gestion des mouvements X et Y du joystick du fauteuil
- **Communication série avec ROS2** : Protocole simplifié pour l'intégration avec Raspberry Pi
- **Fonctions de sécurité** : 
  - Limitation d'amplitude à ±15°
  - Retour automatique à la position neutre après timeout
  - Fonction d'arrêt d'urgence

## Branchement matériel

### Arduino Uno - Brochage

```
JOYSTICK PS2:
- X-axis (Left/Right) → A0 (Analog Input)
- Y-axis (Forward/Backward) → A1 (Analog Input)
- VCC → 5V
- GND → GND

SERVOMOTEURS MG996R:
- Servo X (Horizontal) → Pin 9 (PWM)
- Servo Y (Vertical) → Pin 10 (PWM)
- VCC → 5V (via alimentation externe recommandée)
- GND → GND

ALIMENTATION:
- Arduino Uno via USB (pour tests)
- Servomoteurs via alimentation externe 5V (recommandé pour éviter les chutes de tension)
```

## Protocole de communication série

### Format des commandes (ROS2 → Arduino)

- **SERVO,X,Y** : Positionner les servos aux angles X et Y
  - Exemple: `SERVO,80,95`
  - Réponse: `SERVO_OK,80,95`

- **NEUTRAL** : Retour à la position neutre
  - Réponse: `NEUTRAL_OK`

- **EMERGENCY** : Arrêt d'urgence
  - Réponse: `EMERGENCY_STOP: All servos returned to neutral`

- **STATUS** : Demander l'état actuel
  - Réponse: `STATUS,currentX,currentY,neutralX,neutralY,amplitude`

### Format des données (Arduino → ROS2)

- **JOYSTICK,X,Y** : Position actuelle du joystick
  - Exemple: `JOYSTICK,90,85`
  - Fréquence: 20Hz (toutes les 50ms)

## Configuration ROS2

Pour utiliser ce script avec ROS2 sur Raspberry Pi 3, vous devez configurer un nœud ROS2 qui:

1. **Écoute les messages** du port série (115200 bauds)
2. **Parse les messages** au format `JOYSTICK,X,Y`
3. **Envoie des commandes** au format `SERVO,X,Y`

### Exemple de configuration Python pour ROS2

```python
import rclpy
from rclpy.node import Node
import serial
from custom_msgs.msg import Joystick

class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('arduino_interface')
        
        # Configuration du port série
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        # Publisher pour les données du joystick
        self.joystick_pub = self.create_publisher(Joystick, 'joystick_data', 10)
        
        # Timer pour la lecture périodique
        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz
    
    def read_serial_data(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if line.startswith('JOYSTICK,'):
                parts = line.split(',')
                if len(parts) == 3:
                    msg = Joystick()
                    msg.x_angle = float(parts[1])
                    msg.y_angle = float(parts[2])
                    self.joystick_pub.publish(msg)
```

## Procédure de test

1. **Branchez le matériel** selon le schéma ci-dessus
2. **Téléversez le script** sur l'Arduino Uno
3. **Ouvrez le moniteur série** (115200 bauds) pour vérifier la communication
4. **Testez les commandes manuellement** :
   - Envoyez `STATUS` pour vérifier l'état initial
   - Envoyez `SERVO,80,95` pour tester le mouvement
   - Envoyez `NEUTRAL` pour revenir à la position neutre
   - Envoyez `EMERGENCY` pour tester l'arrêt d'urgence

5. **Intégrez avec ROS2** en utilisant le nœud d'interface

## Dépannage

### Problèmes courants

- **Servos qui tremblent** : Vérifiez l'alimentation externe pour les servos
- **Pas de réponse série** : Vérifiez le câble USB et le port COM
- **Mouvements erratiques** : Vérifiez les connexions du joystick
- **Timeouts fréquents** : Augmentez la valeur `TIMEOUT_MS` si nécessaire

### Messages d'erreur

- `ERROR: Unknown command: ...` : Commande non reconnue
- `TIMEOUT: Returned to neutral position` : Pas de commande reçue pendant 1 seconde
- `EMERGENCY_STOP: ...` : Arrêt d'urgence déclenché

## Notes de sécurité

- **Toujours tester** avec les servos débranchés du fauteuil au début
- **Utilisez une alimentation externe** pour les servos pour éviter les problèmes d'alimentation
- **Ne pas dépasser** les limites d'amplitude (±15°) pour éviter d'endommager le système
- **Gardez toujours** un moyen de couper l'alimentation en cas d'urgence

## Licence

Ce projet est sous licence Apache 2.0 - voir le fichier LICENSE pour plus de détails.