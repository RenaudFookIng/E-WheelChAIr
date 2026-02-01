# Schéma de Câblage - E-WheelChAIr (1 Arduino Mega)

## 📐 Vue d'Ensemble

```
+---------------------+
|   Arduino Mega      |
|                     |
|  A0 ──── Joystick X |
|  A1 ──── Joystick Y |
|  D9 ──── Servo Y    |
|  D10 ─── Servo X    |
|  D2 ──── US4 Trig   |
|  D3 ──── US3 Echo   |
|  D4 ──── US2 Echo   |
|  D5 ──── US1 Trig   |
|  D6 ──── US1 Echo   |
|  D7 ──── US2 Trig   |
|  D8 ──── US3 Trig   |
|  D13 ─── US4 Echo   |
|  GND ─── GND Commun |
|  5V ──── 5V Capteurs|
+---------------------+
       │ USB
       ▼
+---------------------+
|   Raspberry Pi      |
|   (ROS2)            |
+---------------------+
```

## 🔌 Câblage Détailé

### 1. Joystick PS2

```
[Joystick PS2]
   │
   ├── Rouge  → +5V (Arduino)
   ├── Noir   → GND (Arduino)
   ├── Jaune  → A0 (X-axis)
   ├── Vert   → A1 (Y-axis)
   └── Blanc  → (Non utilisé)
```

**Vérification** :
- Mesurer la tension entre Rouge et Noir : doit être ~5V
- Vérifier que X et Y varient entre 0-1023 dans le moniteur série

### 2. Servomoteurs MG996R

```
[Servo X - Gauche/Droite]
   │
   ├── Rouge  → +6V (Alim externe)
   ├── Marron → GND (Alim externe)
   └── Orange → D10 (Arduino)

[Servo Y - Avant/Arrière]
   │
   ├── Rouge  → +6V (Alim externe)
   ├── Marron → GND (Alim externe)
   └── Orange → D9 (Arduino)
```

**IMPORTANT** :
- GND **commun** entre alimentation et Arduino
- Courant max : 2A par servo sous charge

### 3. Capteurs Ultrasons (HC-SR04)

```
[Ultrason 1 - Arrière Droite]
   │
   ├── VCC   → +5V (Arduino)
   ├── Trig  → 30 (Arduino)
   ├── Echo  → 31 (Arduino)
   └── GND   → GND (Arduino)

[Ultrason 2 - Arrière Central]
   │
   ├── VCC   → +5V (Arduino)
   ├── Trig  → 32 (Arduino)
   ├── Echo  → 33 (Arduino)
   └── GND   → GND (Arduino)

[Ultrason 3 - Arrière Gauche]
   │
   ├── VCC   → +5V (Arduino)
   ├── Trig  → 34 (Arduino)
   ├── Echo  → 35 (Arduino)
   └── GND   → GND (Arduino)

```

**Positionnement recommandé** :
- Arrière Droite : Capteur 1
- Arriere Central : Capteur 2
- Arriere Gauche : Capteur 3

### 4. Alimentation

```
[Alimentation 5V Raspberry Pi]
   │
   └── USB  →  Arduino Mega (Port série)
```

### 5. Raspberry Pi

```
[Raspberry Pi]
   │
   └── USB → Arduino Mega (Port série)
```

**Configuration** :
- Port série : `/dev/ttyACM0` (vérifier avec `ls /dev/ttyACM*`)
- Baudrate : 115200
- Permissions : `sudo chmod a+rw /dev/ttyACM0`

## 📊 Tableau de Branchement

| Composant          | Broche Arduino | Broche Composant | Couleur Fil |
|--------------------|----------------|------------------|-------------|
| Joystick X         | A0             | X                | Jaune       |
| Joystick Y         | A1             | Y                | Vert        |
| Servo X (G/D)      | D10            | Signal           | Orange      |
| Servo Y (A/A)      | D9             | Signal           | Orange      |
| Ultrason 1 Trig    | 30             | Trig             | Marron      |
| Ultrason 1 Echo    | 31             | Echo             | Bleu        |
| Ultrason 2 Trig    | 32             | Trig             | Marron      |
| Ultrason 2 Echo    | 33             | Echo             | Bleu        |
| Ultrason 3 Trig    | 34             | Trig             | Marron      |
| Ultrason 3 Echo    | 35             | Echo             | Bleu        |
| Alim 5V            | 5V             | VCC (tous)       | Rouge       |
| GND Commun         | GND            | GND (tous)       | Noir        |

## ⚠️ Vérifications de Sécurité

### Avant Premier Allumage

1. **Vérifier les tensions** :
   ```bash
   # Mesurer entre GND et +5V Arduino
   # Doit être : 4.8V - 5.2V
   ```

2. **Vérifier les connexions** :
   - Pas de court-circuit
   - Tous les GND connectés ensemble
   - Alimentation servos séparée

3. **Tester les servos** :
   ```bash
   # Sans rien brancher au fauteuil
   ros2 topic pub /servo_commands custom_msgs/msg/ServoCommand "{x_normalized: 0.5, y_normalized: 0.0}"
   ```

### Après Branchement

1. **Vérifier les amplitudes** :
   - Max +15° : `ros2 topic pub /servo_commands ... "{x_normalized: 1.0, y_normalized: 1.0}"`
   - Max -15° : `ros2 topic pub /servo_commands ... "{x_normalized: -1.0, y_normalized: -1.0}"`

2. **Tester les ultrasons** :
   ```bash
   ros2 topic echo /ultrasonic_data
   # Doit afficher 3 valeurs entre 0.02 et 4.0
   ```

3. **Vérifier le joystick** :
   ```bash
   ros2 topic echo /joystick_data
   # Doit varier entre -1.0 et 1.0
   ```

## 📱 Dépannage Visuel

### Problème : Servos ne bougent pas

**Causes** :
- ❌ GND non commun
- ❌ Broches servos inversées

**Solution** :
1. Vérifier continuité GND
2. Tester avec exemple Arduino simple

### Problème : Ultrasons toujours à 4.0m

**Causes** :
- ❌ Branchement Trig/Echo inversé
- ❌ Objet trop proche (<2cm)
- ❌ Objet trop éloigné (>4m)

**Solution** :
1. Vérifier branchements
2. Tester avec main à 20cm
3. Vérifier dans moniteur série

### Problème : Joystick non détecté

**Causes** :
- ❌ A0/A1 non branchés
- ❌ Joystick défectueux
- ❌ Alimentation manquante

**Solution** :
1. Mesurer tension joystick
2. Tester avec autre joystick
3. Vérifier valeurs dans moniteur série

## 🎯 Checklist Final

- [ ] Branchements vérifiés (2x)
- [ ] GND commun à tous
- [ ] Code Arduino téléversé
- [ ] ROS2 compilé
- [ ] Ports série configurés
- [ ] Tests unitaires passés
- [ ] Sécurité mécanique OK

## 📚 Schéma Visuel Simplifié

```
+-----------+       +-------------------+       +-----------+
| Joystick  |       |   Arduino Mega    |       | Servos    |
|  PS2      |       |                   |       | MG996R   |
+-----+-----+       +--------+----------+       +-----+-----+
      │                  │ USB              │             │
      │ A0,A1            │                  │ D9,D10      │
      │                  +--------+----------+             │
      │                           │                     │
      └───────────────────┼─────────────────┘
                          │
                          ▼
                    +-------------+
                    | Raspberry Pi|
                    |   ROS2      |
                    +-------------+
```

---

**⚠️ Sécurité** : Toujours tester les servos **avant** connexion au fauteuil.
**🔧 Outils** : Multimètre, tournevis, pince à dénuder.
**✅ Validation** : Vérifier chaque étape avant de passer à la suivante.
