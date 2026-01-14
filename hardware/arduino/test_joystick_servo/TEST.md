# Guide de Test - Joystick & Servo avec ROS2

## Préparation de l'environnement Raspberry Pi 3

Avant de tester le script Arduino, il est essentiel de vérifier que votre environnement ROS2 sur Raspberry Pi 3 est correctement configuré.

### 1. Vérification de l'environnement ROS2

```bash
# Vérifier la version de ROS2 installée
ros2 --version

# Vérifier que colcon est installé
colcon --version

# Vérifier les packages ROS2 disponibles
ros2 pkg list
```

### 2. Vérification des dépendances Python

```bash
# Vérifier que les librairies Python nécessaires sont installées
pip3 list | grep -E 'pyserial|rclpy|numpy'

# Si pyserial manque (nécessaire pour la communication série)
pip3 install pyserial
```

### 3. Vérification du workspace ROS2

```bash
# Aller dans votre workspace ROS2
cd ~/your_workspace

# Vérifier que le build est à jour
colcon build --symlink-install

# Sourcer l'environnement
source install/setup.bash
```

### 4. Vérification des messages personnalisés

Assurez-vous que vos messages personnalisés (`custom_msgs`) sont bien compilés :

```bash
# Vérifier que les messages sont disponibles
ros2 interface list | grep custom_msgs
```

### 5. Test de communication série de base

Avant de connecter l'Arduino, testez que le port série est accessible :

```bash
# Vérifier les ports série disponibles
ls /dev/tty*

# Tester la communication avec un simple script Python
python3 -c "import serial; print(serial.__version__)"
```

## Procédure de test complète

### Étape 1 : Préparation du matériel

1. **Branchez le joystick et les servos** selon le schéma dans `WIRING_SCHEMA.md`
2. **Vérifiez les connexions** avec un multimètre si nécessaire
3. **Utilisez une alimentation externe** pour les servos (recommandé)

### Étape 2 : Téléchargement du script Arduino

1. Ouvrez l'IDE Arduino
2. Chargez le fichier `test_joystick_servo.ino`
3. Sélectionnez le bon port et la carte (Arduino Uno)
4. Téléchargez le script sur l'Arduino

### Étape 3 : Test initial avec le moniteur série

1. **Connectez l'Arduino** au Raspberry Pi via USB
2. **Ouvrez le moniteur série** :
   ```bash
   screen /dev/ttyACM0 115200
   ```
   (Pour quitter : Ctrl+A puis Ctrl+\)

3. **Vérifiez les messages de démarrage** :
   - "E-WheelChAIr Test Controller - Initializing..."
   - "Servos initialized and moved to neutral position"
   - "System ready. Waiting for commands..."

4. **Testez les commandes manuellement** :
   - Envoyez `STATUS` pour vérifier l'état initial
   - Envoyez `SERVO,80,95` pour tester le mouvement
   - Envoyez `NEUTRAL` pour revenir à la position neutre
   - Envoyez `EMERGENCY` pour tester l'arrêt d'urgence

### Étape 4 : Test des données du joystick

1. **Bougez le joystick** et observez les messages :
   - Vous devriez voir des messages comme `JOYSTICK,90,85`
   - La fréquence devrait être d'environ 20Hz (toutes les 50ms)

2. **Vérifiez la plage de valeurs** :
   - X et Y devraient varier entre 0 et 180
   - Les valeurs devraient changer de manière fluide

### Étape 5 : Intégration avec ROS2

1. **Créez un nœud ROS2** pour l'interface (exemple dans README.md)
2. **Lancez le nœud** :
   ```bash
   ros2 run your_package arduino_interface
   ```

3. **Vérifiez les topics** :
   ```bash
   ros2 topic list
   ros2 topic echo /joystick_data
   ```

4. **Testez l'envoi de commandes** depuis ROS2 :
   ```bash
   ros2 topic pub /servo_command std_msgs/msg/String "data: 'SERVO,85,90'"
   ```

## Résolution des problèmes courants

### Problème : Pas de port série détecté

**Solutions** :
- Vérifiez le câble USB
- Essayez un autre port USB
- Redémarrez l'Arduino
- Vérifiez les permissions : `ls -l /dev/ttyACM0`
- Ajoutez votre utilisateur au groupe dialout : `sudo usermod -a -G dialout $USER`

### Problème : Messages corrompus ou illisibles

**Solutions** :
- Vérifiez le baud rate (doit être 115200)
- Vérifiez les connexions GND entre Arduino et Raspberry Pi
- Essayez un autre câble USB
- Ajoutez des condensateurs de découplage près des servos

### Problème : Servos qui tremblent ou mouvements erratiques

**Solutions** :
- Utilisez une alimentation externe pour les servos
- Vérifiez que le GND est bien connecté entre tous les composants
- Réduisez l'amplitude dans le code Arduino (variable `amplitude`)
- Ajoutez un délai plus long dans la boucle principale

### Problème : Timeout fréquent

**Solutions** :
- Augmentez la valeur `TIMEOUT_MS` dans le code Arduino
- Vérifiez que votre nœud ROS2 envoie bien des commandes régulièrement
- Vérifiez qu'il n'y a pas de latence dans la communication série

## Journal de test

Utilisez ce tableau pour noter vos observations pendant les tests :

| Date       | Heure  | Test effectué               | Résultat          | Observations                |
|------------|--------|-----------------------------|-------------------|-----------------------------|
| AAAA-MM-JJ | HH:MM  | Connexion Arduino           | ✓/✗              |                             |
| AAAA-MM-JJ | HH:MM  | Communication série         | ✓/✗              |                             |
| AAAA-MM-JJ | HH:MM  | Mouvement des servos        | ✓/✗              |                             |
| AAAA-MM-JJ | HH:MM  | Lecture du joystick         | ✓/✗              |                             |
| AAAA-MM-JJ | HH:MM  | Intégration ROS2            | ✓/✗              |                             |

## Checklist avant production

- [ ] Tous les composants sont correctement branchés
- [ ] L'alimentation est stable et suffisante
- [ ] La communication série fonctionne sans erreur
- [ ] Les servos répondent correctement aux commandes
- [ ] Le joystick envoie des valeurs cohérentes
- [ ] Le timeout de sécurité fonctionne
- [ ] L'arrêt d'urgence est opérationnel
- [ ] L'intégration ROS2 est fonctionnelle
- [ ] Tous les messages sont correctement formatés
- [ ] Les performances sont satisfaisantes (20Hz)

## Conseils de sécurité

1. **Testez toujours** avec les servos débranchés du fauteuil au début
2. **Utilisez une alimentation externe** pour les servos
3. **Gardez une main sur l'interrupteur** d'urgence
4. **Commencez avec des amplitudes réduites** (±5° au début)
5. **Surveillez la température** des composants
6. **Ayez un plan d'urgence** en cas de comportement inattendu

## Prochaines étapes après validation

Une fois que tous les tests sont réussis :

1. **Intégrez** ce module dans votre système complet
2. **Testez** avec le fauteuil roulant (en environnement sécurisé)
3. **Ajustez** les paramètres si nécessaire (amplitude, timeout, etc.)
4. **Documentez** les paramètres finaux
5. **Passez** à l'intégration des autres composants (ultrasons, etc.)