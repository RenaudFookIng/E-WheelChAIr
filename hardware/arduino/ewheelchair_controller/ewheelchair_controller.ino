/*****************************************************************
  E-WheelChAIr Unified Arduino Controller
  
  DESCRIPTION:
  This Arduino sketch controls the complete hardware interface for the 
  E-WheelChAIr project. It handles:
  - PS2 Joystick input (user control)
  - 2x MG996R Servo motors (wheelchair joystick manipulation)
  - 4x HC-SR04 Ultrasonic sensors (obstacle detection)
  - Serial communication with ROS2 (Raspberry Pi)
  
  HARDWARE: Arduino Mega 2560 (required for sufficient I/O pins)
  
  SAFETY FEATURES:
  - ±15° amplitude limit on servos
  - Automatic timeout return to neutral
  - Emergency stop function
  - Distance clamping for ultrasonic sensors
  
  AUTHOR: E-WheelChAIr Team
  VERSION: 1.0
  DATE: 2024
 *****************************************************************/

// Include the standard Arduino Servo library
#include <Servo.h>

/*****************************************************************
  PIN DEFINITIONS - Hardware Connection Mapping
 *****************************************************************/

// ANALOG INPUTS
#define JOYSTICK_X_PIN A0  // Joystick X-axis (Left/Right)
#define JOYSTICK_Y_PIN A1  // Joystick Y-axis (Forward/Backward)

// DIGITAL OUTPUTS - Servos
#define SERVO_X_PIN 10     // Servo for X-axis (Horizontal movement)
#define SERVO_Y_PIN 9      // Servo for Y-axis (Vertical movement)

// ULTRASONIC SENSORS
// Front Sensor (US1)
#define US1_TRIG 5         // Trigger pin for front ultrasonic sensor
#define US1_ECHO 6         // Echo pin for front ultrasonic sensor

// Rear Sensor (US2)
#define US2_TRIG 7         // Trigger pin for rear ultrasonic sensor
#define US2_ECHO 4         // Echo pin for rear ultrasonic sensor

// Left Sensor (US3)
#define US3_TRIG 8         // Trigger pin for left ultrasonic sensor
#define US3_ECHO 3         // Echo pin for left ultrasonic sensor

// Right Sensor (US4)
#define US4_TRIG 2         // Trigger pin for right ultrasonic sensor
#define US4_ECHO 13        // Echo pin for right ultrasonic sensor

/*****************************************************************
  GLOBAL VARIABLES AND OBJECTS
 *****************************************************************/

// Create servo objects
Servo servoX;  // X-axis servo (Left/Right movement)
Servo servoY;  // Y-axis servo (Forward/Backward movement)

// SERVO CONFIGURATION
// NOTE: Y-axis neutral is 85° (not 90°) to match wheelchair mechanics
int neutralX = 85;    // Neutral position for X servo (degrees)
int neutralY = 90;    // Neutral position for Y servo (degrees)
int currentX = 85;    // Current X position (degrees)
int currentY = 90;    // Current Y position (degrees)
int amplitude = 15;   // Maximum deviation from neutral (±15°)

// SERIAL COMMUNICATION SETTINGS
const long BAUD_RATE = 115200;  // Communication speed with ROS2
unsigned long lastSerialTime = 0;  // Timestamp of last serial transmission
const unsigned long SERIAL_INTERVAL = 50;  // 20Hz update rate (50ms)

// SAFETY FEATURES
unsigned long lastCommandTime = 0;  // Timestamp of last received command
const unsigned long TIMEOUT_MS = 1000;  // 1 second timeout for safety

/*****************************************************************
  SETUP FUNCTION - Runs once at startup
 *****************************************************************/
void setup() {
  // Initialize serial communication with ROS2
  Serial.begin(BAUD_RATE);
  
  // Wait for serial port to connect (important for USB)
  while (!Serial) {
    ; // Busy wait until serial is ready
  }
  
  Serial.println("E-WheelChAIr Controller - Initializing...");
  
  // Initialize servo motors
  servoX.attach(SERVO_X_PIN);  // Attach X servo to pin 10
  servoY.attach(SERVO_Y_PIN);  // Attach Y servo to pin 9
  
  // Move servos to neutral position for safety
  moveToNeutral();
  Serial.println("Servos initialized and moved to neutral position");
  
  // Configure ultrasonic sensor pins
  // Trigger pins are OUTPUT (send ultrasonic pulse)
  // Echo pins are INPUT (receive reflected pulse)
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

/*****************************************************************
  MAIN LOOP - Runs continuously
 *****************************************************************/
void loop() {
  // 1. CHECK FOR INCOMING COMMANDS FROM ROS2
  if (Serial.available() > 0) {
    processSerialCommand();  // Process any received commands
    lastCommandTime = millis();  // Reset safety timeout timer
  }
  
  // 2. SAFETY TIMEOUT CHECK
  // If no commands received for TIMEOUT_MS, return to neutral
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (currentX != neutralX || currentY != neutralY) {
      moveToNeutral();  // Safety: return to neutral position
      Serial.println("TIMEOUT: Returned to neutral position");
    }
  }
  
  // 3. PERIODIC SENSOR DATA TRANSMISSION
  // Send joystick and ultrasonic data at 20Hz (every 50ms)
  if (millis() - lastSerialTime > SERIAL_INTERVAL) {
    sendJoystickData();      // Send current joystick position
    sendUltrasonicData();    // Send all ultrasonic distances
    lastSerialTime = millis();  // Update last transmission time
  }
  
  // 4. SMALL DELAY TO PREVENT CPU OVERLOAD
  delay(10);  // 10ms delay for system stability
}

/*****************************************************************
  COMMAND PROCESSING - Handle incoming serial commands
 *****************************************************************/
void processSerialCommand() {
  // Read complete command line
  String command = Serial.readStringUntil('\n');
  command.trim();  // Remove whitespace
  
  // COMMAND: SERVO,X,Y - Set servo positions
  if (command.startsWith("SERVO,")) {
    // Parse the command: "SERVO,X,Y"
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      // Extract X and Y angles from command
      int xAngle = command.substring(firstComma + 1, secondComma).toInt();
      int yAngle = command.substring(secondComma + 1).toInt();
      
      // SAFETY: Apply amplitude constraints
      // X-axis: 85° ±15° = 70° to 100°
      // Y-axis: 90° ±15° = 75° to 105°
      xAngle = constrain(xAngle, neutralX - amplitude, neutralX + amplitude);
      yAngle = constrain(yAngle, neutralY - amplitude, neutralY + amplitude);
      
      // Move servos to requested positions
      servoX.write(xAngle);
      servoY.write(yAngle);
      
      // Update current positions
      currentX = xAngle;
      currentY = yAngle;
      
      // Send confirmation back to ROS2
      Serial.print("SERVO_OK,");
      Serial.print(currentX);
      Serial.print(",");
      Serial.println(currentY);
    }
  }
  
  // COMMAND: NEUTRAL - Return servos to neutral position
  else if (command.equals("NEUTRAL")) {
    moveToNeutral();
    Serial.println("NEUTRAL_OK");
  }
  
  // COMMAND: STATUS - Request current system status
  else if (command.equals("STATUS")) {
    sendStatus();
  }
  
  // UNKNOWN COMMAND
  else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
  }
}

/*****************************************************************
  MOVEMENT FUNCTIONS
 *****************************************************************/

// Move both servos to neutral position
void moveToNeutral() {
  servoX.write(neutralX);  // X-axis to neutral
  servoY.write(neutralY);  // Y-axis to neutral
  currentX = neutralX;    // Update current X position
  currentY = neutralY;    // Update current Y position
}

// EMERGENCY STOP - Immediately return to neutral
void emergencyStop() {
  moveToNeutral();
  Serial.println("EMERGENCY_STOP: All servos returned to neutral");
}

/*****************************************************************
  SENSOR DATA FUNCTIONS
 *****************************************************************/

// Read and send joystick position data
void sendJoystickData() {
  // Read analog values from joystick (0-1023)
  int xValue = analogRead(JOYSTICK_X_PIN);
  int yValue = analogRead(JOYSTICK_Y_PIN);
  
  // Convert analog values to servo angles (0-180°)
  int xAngle = map(xValue, 0, 1023, 0, 180);
  int yAngle = map(yValue, 0, 1023, 0, 180);
  
  // Send data to ROS2: "JOYSTICK,X,Y"
  Serial.print("JOYSTICK,");
  Serial.print(xAngle);
  Serial.print(",");
  Serial.println(yAngle);
}

// Read and send all ultrasonic sensor distances
void sendUltrasonicData() {
  // Array to store distances from all 4 sensors
  float distances[4];
  
  // Read each sensor sequentially
  distances[0] = readUltrasonicDistance(US1_TRIG, US1_ECHO);  // Front
  distances[1] = readUltrasonicDistance(US2_TRIG, US2_ECHO);  // Rear
  distances[2] = readUltrasonicDistance(US3_TRIG, US3_ECHO);  // Left
  distances[3] = readUltrasonicDistance(US4_TRIG, US4_ECHO);  // Right
  
  // Send data to ROS2: "ULTRASONIC,d1,d2,d3,d4"
  Serial.print("ULTRASONIC,");
  for (int i = 0; i < 4; i++) {
    Serial.print(distances[i], 2);  // 2 decimal places
    if (i < 3) Serial.print(",");   // Comma separator (except last)
  }
  Serial.println();  // Newline to end command
}

// Read distance from a single ultrasonic sensor
float readUltrasonicDistance(int trigPin, int echoPin) {
  // 1. Send 10μs pulse to trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 2. Measure pulse duration on echo pin (timeout after 30ms = ~4m)
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  // 3. Handle timeout (no obstacle detected)
  if (duration == 0) {
    return 4.0;  // Return max distance (4 meters)
  }
  
  // 4. Calculate distance in centimeters
  // Speed of sound = 340 m/s = 0.034 cm/μs
  // Distance = (duration * speed) / 2 (round trip)
  float distance = duration * 0.034 / 2;
  
  // 5. Apply reasonable constraints
  if (distance < 2.0) distance = 2.0;      // Minimum: 2cm
  if (distance > 400.0) distance = 400.0;  // Maximum: 400cm (4m)
  
  // 6. Convert to meters and return
  return distance / 100.0;
}

// Send current system status to ROS2
void sendStatus() {
  // Format: "STATUS,currentX,currentY,neutralX,neutralY,amplitude"
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

/*****************************************************************
  END OF FILE
 *****************************************************************/
// E-WheelChAIr - Making wheelchair control accessible through AI
// Open source project - Apache License 2.0
// https://github.com/your-repo/E-WheelChAIr