#include <Servo.h>

// Joystick
const int pinVRx = A0;
const int pinVRy = A1;

// Ultrasons
const int trigPin1 = 2;
const int echoPin1 = 3;
const int trigPin2 = 4;
const int echoPin2 = 5;

// Servos
Servo servoX;
Servo servoY;
const int servoPinX = 9;
const int servoPinY = 10;

void setup() {
  Serial.begin(115200);
  servoX.attach(servoPinX);
  servoY.attach(servoPinY);

  // Ultrasons
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  // Lecture du joystick
  int valeurX = analogRead(pinVRx);
  int valeurY = analogRead(pinVRy);

  // Envoi des données du joystick
  Serial.print("{\"type\":\"joystick\",\"x\":");
  Serial.print(valeurX);
  Serial.print(",\"y\":");
  Serial.print(valeurY);
  Serial.println("}");

  // Lecture et envoi des ultrasons
  float distance1 = getDistance(trigPin1, echoPin1);
  float distance2 = getDistance(trigPin2, echoPin2);

  Serial.print("{\"type\":\"ultrasonic\",\"sensors\":[");
  Serial.print(distance1);
  Serial.print(",");
  Serial.print(distance2);
  Serial.println("]}");

  // Réception des commandes pour les servos
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("{\"type\":\"servo\",\"angles\":[")) {
      int firstComma = command.indexOf(',');
      int secondComma = command.indexOf(',', firstComma + 1);
      int angleX = command.substring(firstComma + 1, secondComma).toInt();
      int angleY = command.substring(secondComma + 1, command.indexOf(']')).toInt();
      servoX.write(angleX);
      servoY.write(angleY);
    }
  }
  delay(50);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Distance en cm
}
