# Guide de Test Complet - E-WheelChAIr

## 🎯 Objectif

Ce guide vous accompagne pas à pas pour tester l'intégration complète du système avec **1 Arduino Mega** gérant :
- Joystick PS2 (lecture)
- Servos MG996R (contrôle)
- Capteurs ultrasons (à venir)

## 📋 Prérequis

### 1. Matériel Nécessaire

| Composant | Quantité | Branchement |
|-----------|----------|-------------|
| Arduino Mega 2560 | 1 | USB → Raspberry Pi |
| Joystick PS2 | 1 | A0 (X), A1 (Y), D2 (Button) |
| Servo MG996R | 2 | D9 (Y-axis), D10 (X-axis) |
| Alimentation 6V | 1 | Pour les servos |
| Câbles Dupont | 10+ | Connexions diverses |
| Raspberry Pi 4 | 1 | ROS2 + USB |

### 2. Logiciels Installés

- ROS2 Humble
- IDE Arduino 2.x
- Bibliothèques Arduino standard (Servo.h)
- Dépendances Python (pyserial, pyyaml)

## 🔧 Préparation

### 1. Branchements Électroniques

```
[Joystick PS2]
   ├── A0 → Arduino A0 (X-axis)
   ├── A1 → Arduino A1 (Y-axis)
   └── D2 → Arduino D2 (Button - optionnel)

[Arduino Mega]
   ├── D9  → Servo Y-axis (Avant/Arrière)
   ├── D10 → Servo X-axis (Gauche/Droite)
   ├── GND → GND commune
   └── 5V  → Alimentation servos (via régulateur)

[Servos MG996R]
   ├── Rouge  → +6V (alimentation externe)
   ├── Marron → GND
   └── Orange → Signal (D9 ou D10)

[Arduino Mega] → [USB] → [Raspberry Pi]
```

### 2. Téléchargement du Code Arduino

1. Ouvrir `hardware/arduino/servo_controller/servo_controller.ino` dans l'IDE Arduino
2. Sélectionner : `Outils > Carte > Arduino Mega or Mega 2560`
3. Sélectionner le port USB (ex: `/dev/ttyACM0`)
4. Télécharger (Upload) - ✅

### 3. Compilation ROS2

```bash
cd ~/E-WheelChAIr
colcon build --symlink-install
source install/setup.bash
```

## 🧪 Tests par Étapes

### Étape 1 : Test des Servos (Sans Joystick)

**Objectif** : Vérifier que les servos répondent correctement.

**Méthode** :
```bash
# Lancer le node servo_controller
ros2 run servo_controller servo_controller_node

# Envoyer une commande manuelle
ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: 0.0, y_normalized: 0.0}"
```

**Résultats attendus** :
- Servos se positionnent à la position neutre (X=90°, Y=85°)
- Message de confirmation dans le terminal : `SERVO_OK:X=90,Y=85°`

**Tester les amplitudes** :
```bash
# Avant/Droite (X=+15°, Y=+15°)
ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: 1.0, y_normalized: 1.0}"

# Arrière/Gauche (X=-15°, Y=-15°)
ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: -1.0, y_normalized: -1.0}"
```

### Étape 2 : Test du Joystick (Sans ROS)

**Objectif** : Vérifier que le joystick est correctement lu.

**Méthode** :
1. Ouvrir le moniteur série Arduino (115200 bauds)
2. Bouger le joystick dans toutes les directions

**Résultats attendus** :
- Messages `JOYSTICK,X,Y` affichés en continu
- X et Y varient entre 75 et 105 (90±15)
- Retour à 90,85 au centre

### Étape 3 : Test Intégré (Joystick + Servos)

**Objectif** : Vérifier le système complet en mode local.

**Méthode** :
1. Déconnecter le Raspberry Pi
2. Alimenter l'Arduino
3. Bouger le joystick

**Résultats attendus** :
- Servos suivent exactement les mouvements du joystick
- Amplitude limitée à ±15° (sécurité)
- Retour à position neutre si joystick relâché

### Étape 4 : Test ROS2 Complet

**Objectif** : Vérifier l'intégration ROS2.

**Méthode** :
```bash
# Lancer le système complet
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py

# Monitorer les topics
ros2 topic echo /joystick_data
ros2 topic echo /servo_commands
```

**Résultats attendus** :
1. `/joystick_data` : Messages continus avec x,y entre -1.0 et 1.0
2. `/servo_commands` : Commandes servos avec angles correspondants
3. Servos répondent en temps réel aux mouvements du joystick

### Étape 5 : Test de Sécurité

**Objectif** : Vérifier les mécanismes de sécurité.

**Tests** :
1. **Déconnexion USB** : Servos doivent retourner à position neutre
2. **Valeurs hors limite** : Doivent être clampées à ±15°
3. **Timeout** : Après 1s sans commande, retour à neutre

**Commande de test** :
```bash
# Essayer de dépasser les limites (doit être clampé)
ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: 2.0, y_normalized: 2.0}"
```

## 📊 Tableau de Validation

| Test | Résultat Attendu | ✅ Validé |
|------|------------------|-----------|
| Servos neutres | X=90°, Y=85° | ⬜ |
| Amplitude max | ±15° strict | ⬜ |
| Lecture joystick | 75-105 range | ⬜ |
| Communication USB | 115200 bauds | ⬜ |
| Intégration ROS2 | Topics actifs | ⬜ |
| Sécurité timeout | Retour neutre | ⬜ |
| Clamping | Pas de dépassement | ⬜ |

## 🐛 Dépannage

### Problème : Servos ne bougent pas

**Causes possibles** :
- Alimentation insuffisante (servos nécessitent 6V)
- Branchements incorrects
- Port USB non détecté

**Solutions** :
```bash
# Vérifier le port USB
ls /dev/ttyACM*

# Vérifier les permissions
sudo chmod a+rw /dev/ttyACM0

# Tester avec un exemple Arduino simple
```

### Problème : Joystick non détecté

**Causes possibles** :
- Branchements A0/A1 incorrects
- Joystick défectueux
- Alimentation manquante

**Solutions** :
- Vérifier la continuité avec un multimètre
- Tester avec un autre joystick
- Vérifier 5V sur l'Arduino

### Problème : Communication ROS2 échoue

**Causes possibles** :
- Port série incorrect dans la config
- Node ROS2 non lancé
- Conflit de ports

**Solutions** :
```bash
# Vérifier la configuration
cat src/servo_controller/config/servo_config.yaml

# Lister les nodes actifs
ros2 node list

# Redémarrer le node
ros2 run servo_controller servo_controller_node
```

## 📈 Calibration

### 1. Calibration du Joystick

**Méthode** :
1. Lancer le moniteur série
2. Noter les valeurs min/max pour X et Y
3. Ajuster dans le code Arduino si nécessaire

**Valeurs typiques** :
- Centre : X=512, Y=512
- Min : X=0-100, Y=0-100
- Max : X=900-1023, Y=900-1023

### 2. Calibration des Servos

**Méthode** :
1. Positionner les servos manuellement
2. Ajuster les vis de fixation
3. Vérifier l'amplitude mécanique (±15°)

**Outils** :
```bash
# Commande de calibration
ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: 0.5, y_normalized: 0.0}"
```

## ✅ Checklist avant Déploiement

- [ ] Branchements vérifiés (2x)
- [ ] Alimentation stable (6V pour servos)
- [ ] Code Arduino téléversé
- [ ] ROS2 compilé et sourcé
- [ ] Ports série configurés
- [ ] Tests unitaires passés
- [ ] Sécurité mécanique vérifiée
- [ ] Interrupteur d'urgence accessible

## 📚 Documentation Complémentaire

- [Bibliothèque Servo Arduino](https://www.arduino.cc/en/Reference/Servo)
- [ROS2 Troubleshooting](https://docs.ros.org/en/humble/Troubleshooting.html)
- [Guide de Sécurité Électrique](https://learn.sparkfun.com/tutorials/electrical-safety)

---

**Note** : Toujours tester les mouvements des servos **avant** de connecter au joystick du fauteuil.
**Sécurité** : Un adulte doit superviser les tests initiaux.
**Support** : Pour assistance, consulter le README principal ou ouvrir une issue GitHub.
