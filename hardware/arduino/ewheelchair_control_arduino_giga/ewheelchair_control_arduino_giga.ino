/*****************************************************************
  E-WheelChAIr Arduino GIGA Controller (ROS Bridge CSV Version)

  - Lecture 2 axes joystick (X = gauche/droite, Y = avant/arrière)
  - Lecture 3 capteurs ultrasons
  - Reçoit commandes ServoCommand via USB
  - Publie CSV pour ROS : joystick + ultrasons
  - Fréquence 20 Hz
  - Auto-calibration joystick persistante dans Flash
  - Gestion emergency stop
*****************************************************************/

#include <Servo.h>
#include <FlashStorage.h>

// ============================
// Déclaration des servos
// ============================
Servo servoAvantArriere;  // Axe Y
Servo servoGaucheDroite;  // Axe X

// ============================
// Broches Arduino
// ============================
const int brocheVRx = A0;
const int brocheVRy = A1;

const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;

// Ultrasons
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
// Gestion fréquence 20 Hz
// ============================
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 50;

// ============================
// Variables commandes servo
// ============================
int servoX_angle = angleNeutreGaucheDroite;
int servoY_angle = angleNeutreAvantArriere;

// ============================
// Calibration joystick persistante
// ============================
struct JoystickCalibration {
  int centerX;
  int centerY;
  int minX;
  int maxX;
  int minY;
  int maxY;
  bool valid;
};
FlashStorage(joystickStorage, JoystickCalibration);
JoystickCalibration calib;

// ============================
// Prototypes fonctions
// ============================
int readUltrasonic(int trigPin, int echoPin);
void handleIncomingSerial();
void autoCalibrateJoystick();
void saveCalibration();

// ============================
// Setup
// ============================
void setup() {
  Serial.begin(115200);

  // Config ultrasons
  pinMode(trigPin_one, OUTPUT); pinMode(trigEcho_one, INPUT);
  pinMode(trigPin_two, OUTPUT); pinMode(trigEcho_two, INPUT);
  pinMode(trigPin_thr, OUTPUT); pinMode(trigEcho_thr, INPUT);

  // Attache servos
  servoAvantArriere.attach(brocheServoAvantArriere);
  servoGaucheDroite.attach(brocheServoGaucheDroite);

  // Position neutre
  servoAvantArriere.write(angleNeutreAvantArriere);
  servoGaucheDroite.write(angleNeutreGaucheDroite);

  // Charger calibration persistante
  calib = joystickStorage.read();
  if (!calib.valid) {
    Serial.println("⚠️ No calibration found, auto-calibrating joystick...");
    autoCalibrateJoystick();
  }

  Serial.println("READY"); // Pour ROS
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

  // Publication CSV ultrasons
  Serial.print("U,");
  Serial.print(d1); Serial.print(",");
  Serial.print(d2); Serial.print(",");
  Serial.println(d3);

  // -----------------------------
  // Lecture joystick
  // -----------------------------
  int rawX = analogRead(brocheVRx);
  int rawY = analogRead(brocheVRy);

  // Publication CSV joystick (ROS reçoit toujours brutes)
  Serial.print("J,");
  Serial.print(rawX, 6); Serial.print(",");
  Serial.println(rawY, 6);

  // -----------------------------
  // Application des commandes servo depuis ROS
  // -----------------------------
  servoGaucheDroite.write(servoX_angle);
  servoAvantArriere.write(servoY_angle);

  // -----------------------------
  // Lecture commandes ROS
  // -----------------------------
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

  long duree = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  if (duree == 0) return -1;

  return duree * 0.034 / 2; // cm
}

// ============================
// Lecture commandes ROS
// ============================
void handleIncomingSerial() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    // Format CSV attendu: S,x_angle,y_angle,emergency
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
          servoX_angle = angleNeutreGaucheDroite;
          servoY_angle = angleNeutreAvantArriere;
        }
      }
    }
  }
}

// ============================
// Auto-calibration joystick
// ============================
void autoCalibrateJoystick() {
  delay(1000);
  int x = analogRead(brocheVRx);
  int y = analogRead(brocheVRy);

  calib.centerX = x;
  calib.centerY = y;
  calib.minX = x - 400;
  calib.maxX = x + 400;
  calib.minY = y - 400;
  calib.maxY = y + 400;

  saveCalibration();
  Serial.println("✅ Auto-calibration done");
}

// ============================
// Sauvegarde calibration
// ============================
void saveCalibration() {
  calib.valid = true;
  joystickStorage.write(calib);
  Serial.println("✅ Calibration saved to Flash");
}
