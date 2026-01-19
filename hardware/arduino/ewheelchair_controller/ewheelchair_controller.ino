/*****************************************************************
  E-WheelChAIr Unified Arduino Controller (JSON VERSION)
  
  DESCRIPTION:
  Arduino sketch handling the complete hardware interface for the
  E-WheelChAIr project using JSON communication.
  
  FEATURES:
  - PS2 Joystick input
  - 2x MG996R Servo motors
  - 4x HC-SR04 Ultrasonic sensors
  - JSON Serial communication with ROS2
  - Safety timeout & emergency stop
  
  HARDWARE:
  Arduino Mega 2560
  
  AUTHOR: E-WheelChAIr Team
  VERSION: 2.0 (JSON)
  DATE: 2024
 *****************************************************************/

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>

/*****************************************************************
  PIN DEFINITIONS
 *****************************************************************/

// ANALOG INPUTS
#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1

// SERVOS
#define SERVO_X_PIN 10
#define SERVO_Y_PIN 9

// ULTRASONIC SENSORS
#define US1_TRIG 5
#define US1_ECHO 6
#define US2_TRIG 7
#define US2_ECHO 4
#define US3_TRIG 8
#define US3_ECHO 3
#define US4_TRIG 2
#define US4_ECHO 13

/*****************************************************************
  GLOBAL OBJECTS & VARIABLES
 *****************************************************************/

Servo servoX;
Servo servoY;

// SERVO CONFIGURATION
int neutralX = 85;
int neutralY = 90;
int currentX = 85;
int currentY = 90;
int amplitude = 15;

// SERIAL
const long BAUD_RATE = 115200;
unsigned long lastSerialTime = 0;
const unsigned long SERIAL_INTERVAL = 50;

// SAFETY
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 1000;

// JSON
StaticJsonDocument<512> jsonIn;
StaticJsonDocument<512> jsonOut;
String serialBuffer = "";

/*****************************************************************
  SETUP
 *****************************************************************/
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  moveToNeutral();

  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);
  pinMode(US3_TRIG, OUTPUT);
  pinMode(US3_ECHO, INPUT);
  pinMode(US4_TRIG, OUTPUT);
  pinMode(US4_ECHO, INPUT);

  jsonOut["type"] = "ready";
  jsonOut["device"] = "E-WheelChAIr-Arduino";
  serializeJson(jsonOut, Serial);
  Serial.println();
}

/*****************************************************************
  MAIN LOOP
 *****************************************************************/
void loop() {
  processSerialInput();

  if (millis() - lastCommandTime > TIMEOUT_MS) {
    moveToNeutral();
  }

  if (millis() - lastSerialTime > SERIAL_INTERVAL) {
    sendJoystickData();
    sendUltrasonicData();
    lastSerialTime = millis();
  }

  delay(10);
}

/*****************************************************************
  SERIAL JSON INPUT
 *****************************************************************/
void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      DeserializationError error = deserializeJson(jsonIn, serialBuffer);
      if (!error) {
        handleJsonCommand(jsonIn);
        lastCommandTime = millis();
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

/*****************************************************************
  JSON COMMAND HANDLER
 *****************************************************************/
void handleJsonCommand(JsonDocument &doc) {
  const char* type = doc["type"] | "";

  if (strcmp(type, "servo") == 0) {
    bool emergency = doc["emergency"] | false;

    if (emergency) {
      emergencyStop();
      return;
    }

    int xAngle = doc["x_angle"] | neutralX;
    int yAngle = doc["y_angle"] | neutralY;

    xAngle = constrain(xAngle, neutralX - amplitude, neutralX + amplitude);
    yAngle = constrain(yAngle, neutralY - amplitude, neutralY + amplitude);

    servoX.write(xAngle);
    servoY.write(yAngle);
    currentX = xAngle;
    currentY = yAngle;

    sendStatus();
  }

  else if (strcmp(type, "neutral") == 0) {
    moveToNeutral();
    sendStatus();
  }
}

/*****************************************************************
  MOVEMENT FUNCTIONS
 *****************************************************************/
void moveToNeutral() {
  servoX.write(neutralX);
  servoY.write(neutralY);
  currentX = neutralX;
  currentY = neutralY;
}

void emergencyStop() {
  moveToNeutral();
  jsonOut.clear();
  jsonOut["type"] = "emergency";
  jsonOut["status"] = "stopped";
  serializeJson(jsonOut, Serial);
  Serial.println();
}

/*****************************************************************
  SENSOR OUTPUT
 *****************************************************************/
void sendJoystickData() {
  int xRaw = analogRead(JOYSTICK_X_PIN);
  int yRaw = analogRead(JOYSTICK_Y_PIN);

  float xNorm = (xRaw - 512) / 512.0;
  float yNorm = (yRaw - 512) / 512.0;

  jsonOut.clear();
  jsonOut["type"] = "joystick";
  jsonOut["x"] = xNorm;
  jsonOut["y"] = yNorm;

  serializeJson(jsonOut, Serial);
  Serial.println();
}

void sendUltrasonicData() {
  jsonOut.clear();
  jsonOut["type"] = "ultrasonic";

  JsonArray arr = jsonOut.createNestedArray("distances");
  arr.add(readUltrasonicDistance(US1_TRIG, US1_ECHO));
  arr.add(readUltrasonicDistance(US2_TRIG, US2_ECHO));
  arr.add(readUltrasonicDistance(US3_TRIG, US3_ECHO));
  arr.add(readUltrasonicDistance(US4_TRIG, US4_ECHO));

  serializeJson(jsonOut, Serial);
  Serial.println();
}

/*****************************************************************
  ULTRASONIC SENSOR
 *****************************************************************/
float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 4.0;

  float distance = duration * 0.034 / 2.0;
  distance = constrain(distance, 2.0, 400.0);
  return distance / 100.0;
}

/*****************************************************************
  STATUS OUTPUT
 *****************************************************************/
void sendStatus() {
  jsonOut.clear();
  jsonOut["type"] = "status";
  jsonOut["currentX"] = currentX;
  jsonOut["currentY"] = currentY;
  jsonOut["neutralX"] = neutralX;
  jsonOut["neutralY"] = neutralY;
  jsonOut["amplitude"] = amplitude;

  serializeJson(jsonOut, Serial);
  Serial.println();
}

/*****************************************************************
  END OF FILE
 *****************************************************************/
// E-WheelChAIr – JSON-based Arduino Controller
// Open Source – Apache License 2.0
