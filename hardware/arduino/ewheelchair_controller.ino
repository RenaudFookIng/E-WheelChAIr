// E-WheelChAIr Unified Arduino Controller
// Controls joystick, servos, and ultrasonic sensors
// For Arduino Mega 2560

#include <Servo.h>

// Pin Definitions
#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1
#define SERVO_X_PIN 10
#define SERVO_Y_PIN 9

// Ultrasonic Sensor Pins
#define US1_TRIG 5
#define US1_ECHO 6
#define US2_TRIG 7
#define US2_ECHO 4
#define US3_TRIG 8
#define US3_ECHO 3
#define US4_TRIG 2
#define US4_ECHO 13

// Servo Objects
Servo servoX;
Servo servoY;

// Servo Configuration
int neutralX = 90;    // Neutral position for X servo
int neutralY = 85;    // Neutral position for Y servo (85° as per docs)
int currentX = 90;    // Current X position
int currentY = 85;    // Current Y position
int amplitude = 15;   // ±15° amplitude limit

// Serial Communication
const long BAUD_RATE = 115200;
unsigned long lastSerialTime = 0;
const unsigned long SERIAL_INTERVAL = 50; // 20Hz update rate

// Safety
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 1000; // 1 second timeout

void setup() {
  // Initialize Serial Communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("E-WheelChAIr Controller - Initializing...");

  // Initialize Servos
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  moveToNeutral();
  Serial.println("Servos initialized and moved to neutral position");

  // Initialize Ultrasonic Pins
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);
  pinMode(US3_TRIG, OUTPUT);
  pinMode(US3_ECHO, INPUT);
  pinMode(US4_TRIG, OUTPUT);
  pinMode(US4_ECHO, INPUT);
  Serial.println("Ultrasonic sensors initialized");

  Serial.println("System ready. Waiting for commands...");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    processSerialCommand();
    lastCommandTime = millis();
  }

  // Safety timeout - return to neutral if no commands received
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (currentX != neutralX || currentY != neutralY) {
      moveToNeutral();
      Serial.println("TIMEOUT: Returned to neutral position");
    }
  }

  // Periodically send sensor data
  if (millis() - lastSerialTime > SERIAL_INTERVAL) {
    sendJoystickData();
    sendUltrasonicData();
    lastSerialTime = millis();
  }

  // Small delay to prevent CPU overload
  delay(10);
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("SERVO,")) {
    // Parse SERVO,X,Y command
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      int xAngle = command.substring(firstComma + 1, secondComma).toInt();
      int yAngle = command.substring(secondComma + 1).toInt();
      
      // Apply safety constraints
      xAngle = constrain(xAngle, neutralX - amplitude, neutralX + amplitude);
      yAngle = constrain(yAngle, neutralY - amplitude, neutralY + amplitude);
      
      // Move servos
      servoX.write(xAngle);
      servoY.write(yAngle);
      currentX = xAngle;
      currentY = yAngle;
      
      Serial.print("SERVO_OK,");
      Serial.print(currentX);
      Serial.print(",");
      Serial.println(currentY);
    }
  } else if (command.equals("NEUTRAL")) {
    moveToNeutral();
    Serial.println("NEUTRAL_OK");
  } else if (command.equals("STATUS")) {
    sendStatus();
  } else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
  }
}

void moveToNeutral() {
  servoX.write(neutralX);
  servoY.write(neutralY);
  currentX = neutralX;
  currentY = neutralY;
}

void sendJoystickData() {
  int xValue = analogRead(JOYSTICK_X_PIN);
  int yValue = analogRead(JOYSTICK_Y_PIN);
  
  // Convert to servo angles (0-1023 -> 0-180)
  int xAngle = map(xValue, 0, 1023, 0, 180);
  int yAngle = map(yValue, 0, 1023, 0, 180);
  
  Serial.print("JOYSTICK,");
  Serial.print(xAngle);
  Serial.print(",");
  Serial.println(yAngle);
}

void sendUltrasonicData() {
  float distances[4];
  distances[0] = readUltrasonicDistance(US1_TRIG, US1_ECHO);
  distances[1] = readUltrasonicDistance(US2_TRIG, US2_ECHO);
  distances[2] = readUltrasonicDistance(US3_TRIG, US3_ECHO);
  distances[3] = readUltrasonicDistance(US4_TRIG, US4_ECHO);
  
  Serial.print("ULTRASONIC,");
  for (int i = 0; i < 4; i++) {
    Serial.print(distances[i], 2);
    if (i < 3) Serial.print(",");
  }
  Serial.println();
}

float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (4m)
  
  if (duration == 0) {
    return 4.0; // Max distance if timeout
  }
  
  float distance = duration * 0.034 / 2; // Speed of sound: 0.034 cm/μs
  
  // Constrain to reasonable values
  if (distance < 2.0) distance = 2.0; // Minimum distance
  if (distance > 400.0) distance = 400.0; // Maximum distance (4m)
  
  return distance / 100.0; // Convert cm to meters
}

void sendStatus() {
  Serial.print("STATUS,");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.print(",");
  Serial.print(neutralX);
  Serial.print(",");
  Serial.print(neutralY);
  Serial.print(",");
  Serial.println(amplitude);
}

// Emergency stop function
void emergencyStop() {
  moveToNeutral();
  Serial.println("EMERGENCY_STOP: All servos returned to neutral");
}