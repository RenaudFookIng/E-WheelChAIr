#include <Servo.h>

// ============================
// Déclaration des servos
// ============================
Servo servoAvantArriere;  // Axe Y (avant/arrière)
Servo servoGaucheDroite;  // Axe X (gauche/droite)

// ============================
// Broches Arduino
// ============================
const int brocheVRx = A0;
const int brocheVRy = A1;

// ServoM
const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;

// UltraSon (3 capteurs)
const int trigPin_one = 30;  // Capteur 1
const int trigEcho_one = 31;

const int trigPin_two = 32;  // Capteur 2
const int trigEcho_two = 33;

const int trigPin_thr = 34;  // Capteur 3
const int trigEcho_thr = 35;

// ============================
// Paramètres servos
// ============================
const int angleNeutreAvantArriere = 85;
const int angleNeutreGaucheDroite = 90;
const int amplitude = 15;  // ±15° autour de la position neutre

// ============================
// Gestion de la fréquence
// ============================
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 50; // 20 Hz

// ============================
// Setup
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
  int valeurX = analogRead(brocheVRx);
  int valeurY = analogRead(brocheVRy);

  // Mapping joystick sur angles
  int deltaX = map(valeurX, 0, 1023, -amplitude, amplitude);
  int deltaY = map(valeurY, 0, 1023, -amplitude, amplitude);

  int angleX = constrain(angleNeutreGaucheDroite + deltaX,
                         angleNeutreGaucheDroite - amplitude,
                         angleNeutreGaucheDroite + amplitude);
  int angleY = constrain(angleNeutreAvantArriere + deltaY,
                         angleNeutreAvantArriere - amplitude,
                         angleNeutreAvantArriere + amplitude);

  // -----------------------------
  // Commande servos
  // -----------------------------
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
