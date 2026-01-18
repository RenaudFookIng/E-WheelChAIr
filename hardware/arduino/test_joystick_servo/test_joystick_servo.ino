#include <Servo.h>
#include <ArduinoJson.h>  // Bibliothèque pour JSON (Installer via Library Manager)

// ==============================
//         CONFIGURATION
// ==============================

#define JOY_X A0   // Joystick X
#define JOY_Y A1   // Joystick Y

#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10

#define SERVO_MIN 0
#define SERVO_MAX 180

Servo servoX;
Servo servoY;

// Temps entre chaque publication sur le port série (ms)
const unsigned long SERIAL_INTERVAL = 50;  // 20 Hz

unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(115200);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Servos au neutre
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  // =========================
  // LECTURE JOYSTICK
  // =========================
  int rawX = analogRead(JOY_X);  // 0-1023
  int rawY = analogRead(JOY_Y);  // 0-1023

  // Normalisation -1.0 → 1.0
  float normX = map(rawX, 0, 1023, -1000, 1000) / 1000.0;
  float normY = map(rawY, 0, 1023, -1000, 1000) / 1000.0;

  // =========================
  // COMMANDES SERVOS
  // =========================
  int angleX = map(normX * 1000, -1000, 1000, SERVO_MIN, SERVO_MAX);
  int angleY = map(normY * 1000, -1000, 1000, SERVO_MIN, SERVO_MAX);

  // Écriture servo
  servoX.write(angleX);
  servoY.write(angleY);

  // =========================
  // ENVOI JSON SUR SÉRIE
  // =========================
  unsigned long now = millis();
  if (now - lastSerialTime > SERIAL_INTERVAL) {
    lastSerialTime = now;

    StaticJsonDocument<200> doc;
    doc["type"] = "joystick";
    doc["x"] = normX;
    doc["y"] = normY;

    serializeJson(doc, Serial);
    Serial.println();
  }

  delay(10);  // Petite pause pour stabilité
}
