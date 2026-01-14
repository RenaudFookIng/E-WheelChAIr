# Guide d'Installation et de Dépendances - E-WheelChAIr

## 📋 Prérequis Système

### 1. Système d'Exploitation
- **Recommandé** : Ubuntu 20.04/22.04 LTS (64-bit)
- **Alternative** : Raspberry Pi OS (64-bit) pour Raspberry Pi 4/5
- **Espace disque** : 20GB minimum
- **Mémoire** : 4GB RAM minimum (8GB recommandé)

### 2. ROS2 Installation

#### Installation de ROS2 Humble (recommandé)
```bash
# Configuration des dépôts
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Installation
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Configuration de l'environnement
source /opt/ros/humble/setup.bash
```

#### Vérification
```bash
source /opt/ros/humble/setup.bash
ros2 --version
# Doit afficher : humble
```

## 🔧 Dépendances ROS2

### Dépendances Python
```bash
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install pyserial pyyaml setuptools
```

### Dépendances pour la compilation
```bash
sudo apt install python3-colcon-common-extensions 
                     python3-rosdep2 
                     build-essential 
                     cmake 
                     git

# Initialisation rosdep
sudo rosdep init
rosdep update
```

## 🌍 Dépendances Arduino

### 1. IDE Arduino
```bash
# Installation officielle
sudo apt install arduino

# Ou version plus récente depuis le site Arduino
wget https://downloads.arduino.cc/arduino-ide/2.3.2/linux/arduino-ide_2.3.2_Linux_64bit.AppImage
chmod +x arduino-ide_2.3.2_Linux_64bit.AppImage
./arduino-ide_2.3.2_Linux_64bit.AppImage
```

### 2. Bibliothèques Arduino Standard
Les bibliothèques suivantes sont **incluses avec l'IDE Arduino** :
- `Servo.h` - Contrôle des servomoteurs
- `Wire.h` - Communication I2C
- `SPI.h` - Communication SPI
- `SoftwareSerial.h` - Série logicielle
- `EEPROM.h` - Mémoire EEPROM

**Vérification** :
1. Ouvrir l'IDE Arduino
2. `Fichier > Exemples > Servo > Knob`
3. Si l'exemple s'ouvre, la bibliothèque Servo est installée

### 3. Pilotes USB
Pour les cartes Arduino (Uno, Mega, etc.) :
```bash
# Ajouter l'utilisateur au groupe dialout
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Redémarrer la session ou le système
newgrp dialout
```

## 🛠️ Dépendances Matérielles

### 1. Composants Électroniques
| Composant | Quantité | Rôle |
|-----------|----------|------|
| Arduino Uno/Nano | 1 | Lecture du joystick |
| Arduino Mega | 1 | Contrôle des servos |
| Servo MG996R | 2 | Actionneurs mécaniques |
| Joystick PS2 | 1 | Interface utilisateur |
| Câbles USB | 2 | Communication série |
| Alimentation 6V | 1 | Pour les servos |

### 2. Branchements
```
[Joystick PS2] → [Arduino Uno] (A0, A1, D2)
[Arduino Uno] → [Raspberry Pi] (USB - /dev/ttyACM0)
[Raspberry Pi] → [Arduino Mega] (USB - /dev/ttyACM1)
[Arduino Mega] → [Servo X] (D10)
[Arduino Mega] → [Servo Y] (D9)
[Servos] → [Joystick Fauteuil] (Mécanique)
```

## 🚀 Compilation et Installation

### 1. Cloner le dépôt (si pas déjà fait)
```bash
cd ~
git clone https://github.com/votre-depot/E-WheelChAIr.git
cd E-WheelChAIr
```

### 2. Installer les dépendances ROS2
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compiler le workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Vérifier l'installation
```bash
# Lister les packages
ros2 pkg list | grep e_wheelchair

# Vérifier les exécutables
ls install/servo_controller/lib/servo_controller/
ls install/teleop_joystick/lib/teleop_joystick/
```

## ⚙️ Configuration

### 1. Configurer les ports série
Éditer les fichiers de configuration :
- `src/servo_controller/config/servo_config.yaml` (port: "/dev/ttyACM1")
- `src/teleop_joystick/config/joystick_config.yaml` (port: "/dev/ttyACM0")

**Trouver les ports** :
```bash
ls /dev/ttyACM*
```

### 2. Télécharger le code Arduino
1. Ouvrir `hardware/arduino/joystick/joystick.ino` dans l'IDE Arduino
2. Sélectionner la carte (Arduino Uno)
3. Sélectionner le port (/dev/ttyACM0 ou COMx)
4. Télécharger (Upload)

5. Répéter pour `hardware/arduino/servo_controller/servo_controller_ros.ino` (Arduino Mega, /dev/ttyACM1)

## 🎯 Lancement du Système

### 1. Lancer le système complet
```bash
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

### 2. Vérifier les nodes
```bash
ros2 node list
# Doit afficher :
# /master_node
# /servo_controller
# /teleop_joystick
# /lidar
# /visualization
```

### 3. Monitorer les topics
```bash
# Données du joystick
ros2 topic echo /joystick_data

# Commandes des servos
ros2 topic echo /servo_commands

# Statut des servos
ros2 topic echo /servo_status
```

## 🐛 Dépannage

### Problèmes Courants

1. **Permission denied sur les ports série** :
   ```bash
   sudo chmod a+rw /dev/ttyACM0
   sudo chmod a+rw /dev/ttyACM1
   ```

2. **Ports série non détectés** :
   - Vérifier les branchements USB
   - Redémarrer les Arduino
   - Vérifier avec `lsusb`

3. **Erreurs de compilation ROS2** :
   ```bash
   rm -rf build install log
   colcon build --symlink-install
   ```

4. **Servos ne bougent pas** :
   - Vérifier l'alimentation des servos
   - Tester avec un exemple Arduino simple
   - Vérifier les branchements

## 📚 Documentation Supplémentaire

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Arduino Reference](https://www.arduino.cc/reference/)
- [Bibliothèque Servo](https://www.arduino.cc/en/Reference/Servo)

## ✅ Checklist avant premier lancement

- [ ] ROS2 installé et fonctionnel
- [ ] Dépendances Python installées
- [ ] IDE Arduino installé
- [ ] Pilotes USB configurés
- [ ] Code Arduino téléversé
- [ ] Ports série configurés
- [ ] Alimentation des servos connectée
- [ ] Branchements mécaniques sécurisés
- [ ] Interrupteur d'urgence accessible

---

**Support** : Pour toute question, consulter la documentation ou ouvrir une issue sur le dépôt GitHub.
**Sécurité** : Toujours tester les mouvements des servos avant de connecter au joystick du fauteuil.
**Licence** : Ce projet est sous licence Apache 2.0 - voir LICENSE pour plus de détails.
