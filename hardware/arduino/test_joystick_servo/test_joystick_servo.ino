#include <Servo.h>
#include <ArduinoJson.h>

// ==============================
//        CONFIGURATION
// ==============================

#define JOY_X A0   // Joystick X
#define JOY_Y A1   // Joystick Y

#define US_LEFT A2   // Capteur US gauche
#define US_RIGHT A3  // Capteur US droite

#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10

#define SERVO_MIN 0
#define SERVO_MAX 180

Servo servoX;
Servo servoY;

// Fréquence publication série (ms)
const unsigned long SERIAL_INTERVAL = 50;
unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(115200);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Position neutre
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  unsigned long now = millis();

  // =========================
  // 1. Lecture commandes ROS
  // =========================
  if (Serial.available() > 0) {
    String jsonStr = Serial.readStringUntil('\n');
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (!error) {
      int angleX = doc["x_angle"] | 90;
      int angleY = doc["y_angle"] | 90;
      angleX = constrain(angleX, 0, 180);
      angleY = constrain(angleY, 0, 180);

      servoX.write(angleX);
      servoY.write(angleY);
    }
  }

  // =========================
  // 2. Lire Joystick + Ultrasons et publier JSON
  // =========================
  if (now - lastSerialTime > SERIAL_INTERVAL) {
    lastSerialTime = now;

    // Joystick
    int rawX = analogRead(JOY_X);
    int rawY = analogRead(JOY_Y);
    float normX = map(rawX, 0, 1023, -1000, 1000) / 1000.0;
    float normY = map(rawY, 0, 1023, -1000, 1000) / 1000.0;

    // Ultrasons
    int usLeft = analogRead(US_LEFT);   // à adapter selon ton capteur
    int usRight = analogRead(US_RIGHT);

    // Conversion approximative : 0-1023 → 0-200 cm
    float usLeftCm = map(usLeft, 0, 1023, 0, 200);
    float usRightCm = map(usRight, 0, 1023, 0, 200);

    // JSON
    StaticJsonDocument<256> doc;
    doc["type"] = "sensor_data";
    doc["joystick"]["x"] = normX;
    doc["joystick"]["y"] = normY;
    // CORRECTION : créer un tableau JSON
    JsonArray arr = doc.createNestedArray("ultrasonic");
    arr.add(usLeftCm);
    arr.add(usRightCm);

    serializeJson(doc, Serial);
    Serial.println();
  }

  delay(10);
}
