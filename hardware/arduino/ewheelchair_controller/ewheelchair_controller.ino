/*****************************************************************
  E-WheelChAIr Arduino Controller (ROS Bridge CSV Version)
  
  Description :
  -------------
  - Contrôle 2 servos (X = gauche/droite, Y = avant/arrière)
  - Lit 3 capteurs ultrasons
  - Reçoit commandes ROS via USB (CSV)
  - Émet distances ultrasons et positions joystick sur USB
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

const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;

// Ultrasons (3 capteurs)
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

// ============================
// Gestion de la fréquence
// ============================
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 50; // 20 Hz

// ============================
// Variables commandes servo
// ============================
int servoX_angle = angleNeutreGaucheDroite;
int servoY_angle = angleNeutreAvantArriere;

// ============================
// Setup
// ============================
void setup() {
  Serial.begin(115200); // Flux série rapide

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
  // Lecture joystick
  // -----------------------------
  // Valeurs minimales et maximales mesurées pour ton joystick
  const int JOY_MIN_X = 200;
  const int JOY_MAX_X = 750;
  const int JOY_MIN_Y = 200;
  const int JOY_MAX_Y = 750;

  // Lecture joystick
  int valeurX = analogRead(brocheVRx);
  int valeurY = analogRead(brocheVRy);

  // Normalisation sur [-1, 1]
  float normX = 2.0 * (valeurX - JOY_MIN_X) / (JOY_MAX_X - JOY_MIN_X) - 1.0;
  float normY = 2.0 * (valeurY - JOY_MIN_Y) / (JOY_MAX_Y - JOY_MIN_Y) - 1.0;

  // Limite au cas où le joystick sortirait légèrement de la plage
  if (normX > 1.0) normX = 1.0;
  if (normX < -1.0) normX = -1.0;
  if (normY > 1.0) normY = 1.0;
  if (normY < -1.0) normY = -1.0;
  Serial.print("J,");
  Serial.print(normX, 6); Serial.print(",");
  Serial.println(normY, 6);

  // -----------------------------
  // Application des commandes servo depuis ROS
  // -----------------------------
  servoGaucheDroite.write(servoX_angle);
  servoAvantArriere.write(servoY_angle);

  // Traitement série entrante pour lire commandes ROS
  handleIncomingSerial();
}

// ============================
// Lecture ultrasons
// ============================
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  if (duree == 0) return -1; // Si pas de mesure

  return duree * 0.034 / 2; // Conversion cm
}

// ============================
// Lecture commandes ROS
// ============================
void handleIncomingSerial() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    // Exemple CSV attendu: S,angleX,angleY,emergency
    if (line.startsWith("S,")) {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > 0) {
        String sx = line.substring(firstComma + 1, secondComma);
        String sy;
        if (thirdComma > 0) sy = line.substring(secondComma + 1, thirdComma);
        else sy = line.substring(secondComma + 1);

        servoX_angle = sx.toInt();
        servoY_angle = sy.toInt();
      }

      // Emergency optionnel
      if (thirdComma > 0) {
        String emergencyStr = line.substring(thirdComma + 1);
        bool emergency = emergencyStr.toInt() > 0;
        if (emergency) {
          // En cas d'urgence, revenir à position neutre
          servoX_angle = angleNeutreGaucheDroite;
          servoY_angle = angleNeutreAvantArriere;
        }
      }
    }
  }
}
