#include <Servo.h>
#include <ArduinoJson.h>

// ==============================
//         CONFIGURATION
// ==============================

// Joystick
#define JOY_X A0
#define JOY_Y A1

// Servos
#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10
#define SERVO_MIN 0
#define SERVO_MAX 180

Servo servoX;
Servo servoY;

// Ultrasons (HC-SR04)
#define US_LEFT_TRIG 2
#define US_LEFT_ECHO 3
#define US_RIGHT_TRIG 4
#define US_RIGHT_ECHO 5

// Temps entre publications JSON
const unsigned long SERIAL_INTERVAL = 50;  // ms
unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(115200);

  // Servos au neutre
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(90);
  servoY.write(90);

  // Ultrasons
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
}

// ======================================
//            MESURE ULTRASONS
// ======================================
long readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  long distance = duration * 0.0343 / 2;
  if (distance == 0) distance = 300; // si timeout
  return distance;
}

// ======================================
//               LOOP
// ======================================
void loop() {
  unsigned long now = millis();

  // ===== Lecture Joystick =====
  int rawX = analogRead(JOY_X); // 0-1023
  int rawY = analogRead(JOY_Y); // 0-1023
  float normX = map(rawX, 0, 1023, -1000, 1000) / 1000.0;
  float normY = map(rawY, 0, 1023, -1000, 1000) / 1000.0;

  // ===== Lecture Ultrasons =====
  long usLeftCm = readUltrasonicCm(US_LEFT_TRIG, US_LEFT_ECHO);
  long usRightCm = readUltrasonicCm(US_RIGHT_TRIG, US_RIGHT_ECHO);

  // ===== Publication JSON vers ROS =====
  if (now - lastSerialTime > SERIAL_INTERVAL) {
    lastSerialTime = now;

    StaticJsonDocument<256> doc;
    doc["type"] = "sensor_data";
    doc["joystick"]["x"] = normX;
    doc["joystick"]["y"] = normY;

    JsonArray usArray = doc.createNestedArray("ultrasonic");
    usArray.add(usLeftCm);
    usArray.add(usRightCm);

    serializeJson(doc, Serial);
    Serial.println();
  }

  // ===== Lecture commandes Servo depuis ROS =====
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    StaticJsonDocument<128> cmdDoc;
    DeserializationError error = deserializeJson(cmdDoc, line);
    if (error) continue;

    if (cmdDoc.containsKey("servo_command")) {
      int angleX = cmdDoc["servo_command"]["x_angle"];
      int angleY = cmdDoc["servo_command"]["y_angle"];
      angleX = constrain(angleX, SERVO_MIN, SERVO_MAX);
      angleY = constrain(angleY, SERVO_MIN, SERVO_MAX);
      servoX.write(angleX);
      servoY.write(angleY);
    }
  }

  delay(10); // petite pause
}
