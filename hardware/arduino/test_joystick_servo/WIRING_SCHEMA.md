# Schéma de câblage - Test Joystick & Servo

## Vue d'ensemble du branchement

```
┌───────────────────────────────────────────────────────────────┐
│                    E-WheelChAIr Test System                    │
├───────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐       ┌─────────────┐       ┌─────────────┐  │
│  │  PS2        │       │  Arduino    │       │  Servo X    │  │
│  │  Joystick   │       │    Uno      │       │  (Horizontal)│  │
│  └─────────────┘       └─────────────┘       └─────────────┘  │
│       │                      │                      │          │
│       │                      │                      │          │
│  ┌────┴─────┐          ┌─────┴─────┐          ┌─────┴─────┐  │
│  │ VCC  5V  │─────────▶│ 5V        │          │ VCC  5V  │  │
│  │ GND  GND │─────────▶│ GND       │          │ GND  GND │  │
│  │ X    A0  │─────────▶│ A0 (Analog)│          │ PWM  9   │◀─┘
│  │ Y    A1  │─────────▶│ A1 (Analog)│          │          │
│  └──────────┘          └─────────────┘          └──────────┘  │
│                                                               │
│  ┌─────────────┐       ┌─────────────┐                    │  │
│  │  Servo Y     │       │  Power      │                    │  │
│  │ (Vertical)   │       │  Supply     │                    │  │
│  └─────────────┘       └─────────────┘                    │  │
│       │                      │                           │  │
│       │                      │                           │  │
│  ┌────┴─────┐          ┌─────┴─────┐                    │  │
│  │ VCC  5V  │◀─────────┤ 5V        │                    │  │
│  │ GND  GND │─────────▶│ GND       │                    │  │
│  │ PWM  10  │◀─────────┤ 10 (PWM)  │                    │  │
│  └──────────┘          └─────────────┘                    │  │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

## Liste des composants nécessaires

1. **Arduino Uno** - 1 unité
2. **Joystick PS2** - 1 unité
3. **Servomoteurs MG996R** - 2 unités
4. **Alimentation externe 5V** - 1 unité (pour les servos)
5. **Câbles de connexion** - Assortiment
6. **Breadboard** - 1 unité (optionnel, pour prototypage)

## Connexions détaillées

### Joystick PS2 → Arduino Uno

| Joystick Pin | Arduino Pin | Couleur suggérée | Description          |
|--------------|-------------|------------------|----------------------|
| VCC          | 5V          | Rouge            | Alimentation 5V      |
| GND          | GND         | Noir             | Masse                 |
| X            | A0          | Jaune           | Axe X (Gauche/Droite)|
| Y            | A1          | Vert            | Axe Y (Avant/Arrière) |

### Servo X (Horizontal) → Arduino Uno

| Servo Pin | Arduino Pin | Couleur suggérée | Description          |
|-----------|-------------|------------------|----------------------|
| VCC       | 5V (ext)    | Rouge            | Alimentation 5V      |
| GND       | GND         | Noir             | Masse                 |
| PWM       | 9           | Orange           | Signal de contrôle    |

### Servo Y (Vertical) → Arduino Uno

| Servo Pin | Arduino Pin | Couleur suggérée | Description          |
|-----------|-------------|------------------|----------------------|
| VCC       | 5V (ext)    | Rouge            | Alimentation 5V      |
| GND       | GND         | Noir             | Masse                 |
| PWM       | 10          | Jaune           | Signal de contrôle    |

## Alimentation

### Option 1: Alimentation USB (pour tests initiaux)
- **Arduino Uno** : Alimenté via USB depuis le Raspberry Pi
- **Servos** : Alimentés via le 5V de l'Arduino (limité en courant)
- **Limitation** : Peut causer des chutes de tension avec les servos MG996R

### Option 2: Alimentation externe (recommandée)
- **Arduino Uno** : Alimenté via USB pour la communication série
- **Servos** : Alimentés via une source externe 5V (2A minimum)
- **Connexion** : Relier les GND ensemble (GND commun)

```
┌─────────────┐       ┌─────────────┐
│  Power      │       │  Arduino    │
│  Supply     │       │    Uno      │
└─────────────┘       └─────────────┘
      │                    │
      │                    │
      ▼                    ▼
┌─────────────┐       ┌─────────────┐
│ 5V  ───────┐       │ 5V (USB)    │
│ GND  ───────┼───────▶ GND          │
└─────────────┘       └─────────────┘
      │
      ▼
┌─────────────┐
│ Servo X & Y │
│ VCC & GND   │
└─────────────┘
```

## Configuration des servos

### Positions neutres par défaut
- **Servo X (Horizontal)** : 85°
- **Servo Y (Vertical)** : 90°

### Limites de mouvement
- **Amplitude maximale** : ±15° par rapport à la position neutre
- **Servo X** : 70° à 100°
- **Servo Y** : 75° à 105°

## Vérification du câblage

1. **Vérifiez les connexions** avant d'alimenter
2. **Utilisez un multimètre** pour vérifier la continuité
3. **Testez avec le moniteur série** avant de connecter les servos au fauteuil
4. **Commencez avec une amplitude réduite** pour les premiers tests

## Schéma visuel (représentation textuelle)

```
          +─────────────────────────────────────────────────+
          │                 ARDUINO UNO                      │
          +─────────────────────────────────────────────────+
          │                                             │
          │  A0 ────┐                                   │
          │  A1 ────┼─── JOYSTICK PS2                   │
          │  GND───┐│                                   │
          │  5V────┘│                                   │
          │         │                                   │
          │  D9 ────┐                                   │
          │  D10───┼─── SERVO X & Y                     │
          │  GND───┐│                                   │
          │  5V────┘│                                   │
          │         │                                   │
          │  USB────┬─── RASPBERRY PI 3 (ROS2)          │
          │  GND────┴─── GND (COMMUN)                   │
          +─────────────────────────────────────────────────+
```

## Notes importantes

- **Toujours connecter le GND** entre l'Arduino, les servos et l'alimentation externe
- **Éviter les boucles de masse** pour réduire le bruit électrique
- **Utiliser des condensateurs de découplage** (100nF) près des servos si nécessaire
- **Tester progressivement** : commencez sans charge, puis ajoutez les servos, enfin le système complet