/*****************************************************************
  E-WheelChAIr Arduino Controller (ROS Bridge CSV Version)
  
  Description :
  -------------
  - Contrôle 2 servos (X = gauche/droite, Y = avant/arrière)
  - Lit 3 capteurs ultrasons
  - Reçoit commandes ROS via USB (CSV)
  - Émet distances ultrasons et positions servos sur USB
  - Fréquence fixe à 20 Hz
  - Gestion d’urgence (emergency stop)
  
  Auteur : E-WheelChAIr Team
  Date   : 2026
*****************************************************************/

#include <Servo.h>

// ============================
// Déclaration des servos
// ============================
Servo servoAvantArriere;  // Axe Y (avant/arrière)
Servo servoGaucheDroite;  // Axe X (gauche/droite)

// ============================
// Broches Arduino
// ============================
const int brocheVRx = A0;  // Joystick X
const int brocheVRy = A1;  // Joystick Y

const int brocheServoAvantArriere = 9;  // Servo Y
const int brocheServoGaucheDroite = 10; // Servo X

// Capteurs ultrasons (3 capteurs)
const int trigPin_one = 30; 
const int trigEcho_one = 31;

const int trigPin_two = 32;
const int trigEcho_two = 33;

const int trigPin_thr = 34;
const int trigEcho_thr = 35;

// ============================
// Paramètres servos
// ============================
const int angleNeutreAvantArriere = 85;
const int angleNeutreGaucheDroite = 90;
const int amplitude = 15;  // ±15° autour de la position neutre

// ============================
// Variables commande ROS
// ============================
int cmdAngleX = angleNeutreGaucheDroite;
int cmdAngleY = angleNeutreAvantArriere;
bool emergencyStop = false;

// ============================
// Gestion fréquence 20 Hz
// ============================
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 50; // 20 Hz

// ============================
// Setup Arduino
// ============================
void setup() {
  Serial.begin(115200); // Flux série rapide pour 20Hz

  // Config capteurs ultrasons
  pinMode(trigPin_one, OUTPUT); pinMode(trigEcho_one, INPUT);
  pinMode(trigPin_two, OUTPUT); pinMode(trigEcho_two, INPUT);
  pinMode(trigPin_thr, OUTPUT); pinMode(trigEcho_thr, INPUT);

  // Attache les servos
  servoAvantArriere.attach(brocheServoAvantArriere);
  servoGaucheDroite.attach(brocheServoGaucheDroite);

  // Position neutre
  servoAvantArriere.write(angleNeutreAvantArriere);
  servoGaucheDroite.write(angleNeutreGaucheDroite);

  Serial.println("READY"); // Message initial pour debug
}

// ============================
// Loop principal
// ============================
void loop() {
  // Gestion de la fréquence 20Hz
  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime < LOOP_INTERVAL_MS) return;
  lastLoopTime = currentTime;

  // -----------------------------
  // Lecture ultrasons
  // -----------------------------
  int d1 = readUltrasonic(trigPin_one, trigEcho_one);
  int d2 = readUltrasonic(trigPin_two, trigEcho_two);
  int d3 = readUltrasonic(trigPin_thr, trigEcho_thr);

  // Envoi distances sur série (CSV)
  Serial.print("U,"); // Type U = Ultrasons
  Serial.print(d1); Serial.print(",");
  Serial.print(d2); Serial.print(",");
  Serial.println(d3);

  // -----------------------------
  // Lecture commandes ROS (USB CSV)
  // -----------------------------
  readSerialCommand();

  // -----------------------------
  // Appliquer commandes au servo
  // -----------------------------
  int angleX = cmdAngleX;
  int angleY = cmdAngleY;

  if (emergencyStop) {
    angleX = angleNeutreGaucheDroite;
    angleY = angleNeutreAvantArriere;
  }

  servoGaucheDroite.write(angleX);
  servoAvantArriere.write(angleY);

  // Envoi angles servos sur série (CSV)
  Serial.print("S,"); // Type S = Servo
  Serial.print(angleX); Serial.print(",");
  Serial.println(angleY);
}

// ============================
// Fonction lecture ultrason
// ============================
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  if (duree == 0) return -1; // Si pas de mesure

  return duree * 0.034 / 2; // Conversion en cm
}

// ============================
// Fonction lecture commande ROS série
// Format CSV attendu : S,angleX,angleY,emergency
// ============================
void readSerialCommand() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim(); // Supprime espaces / retour chariot

    if (line.length() == 0) continue;

    if (line[0] == 'S') {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        cmdAngleX = line.substring(firstComma + 1, secondComma).toInt();
        if (thirdComma > secondComma) {
          cmdAngleY = line.substring(secondComma + 1, thirdComma).toInt();
          emergencyStop = line.substring(thirdComma + 1).toInt() != 0;
        } else {
          cmdAngleY = line.substring(secondComma + 1).toInt();
          emergencyStop = false;
        }

        // Limite sécurité
        cmdAngleX = constrain(cmdAngleX, angleNeutreGaucheDroite - amplitude, angleNeutreGaucheDroite + amplitude);
        cmdAngleY = constrain(cmdAngleY, angleNeutreAvantArriere - amplitude, angleNeutreAvantArriere + amplitude);
      }
    }
  }
}
