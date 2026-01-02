/*****************************************************************
  E-WheelChAIr - Test Joystick & Servo Controller
  
  DESCRIPTION:
  Simplified Arduino sketch for testing joystick and servo functionality
  with Arduino Uno and ROS2 on Raspberry Pi 3.
  
  This version focuses on:
  - PS2 Joystick input (user control)
  - 2x MG996R Servo motors (wheelchair joystick manipulation)
  - Serial communication with ROS2 (Raspberry Pi)
  
  HARDWARE: Arduino Uno (optimized for limited I/O pins)
  
  SAFETY FEATURES:
  - ±15° amplitude limit on servos
  - Automatic timeout return to neutral
  - Emergency stop function
  
  AUTHOR: E-WheelChAIr Team
  VERSION: 1.0 - Simplified Test Version
  DATE: 2024
 *****************************************************************/

// Include the standard Arduino Servo library
#include <Servo.h>

/*****************************************************************
  PIN DEFINITIONS - Hardware Connection Mapping for Arduino Uno
 *****************************************************************/

// ANALOG INPUTS
#define JOYSTICK_X_PIN A0  // Joystick X-axis (Left/Right)
#define JOYSTICK_Y_PIN A1  // Joystick Y-axis (Forward/Backward)

// DIGITAL OUTPUTS - Servos
#define SERVO_X_PIN 9      // Servo for X-axis (Horizontal movement)
#define SERVO_Y_PIN 10     // Servo for Y-axis (Vertical movement)

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
  
  Serial.println("E-WheelChAIr Test Controller - Initializing...");
  
  // Initialize servo motors
  servoX.attach(SERVO_X_PIN);  // Attach X servo to pin 9
  servoY.attach(SERVO_Y_PIN);  // Attach Y servo to pin 10
  
  // Move servos to neutral position for safety
  moveToNeutral();
  Serial.println("Servos initialized and moved to neutral position");
  
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
  
  // 3. PERIODIC JOYSTICK DATA TRANSMISSION
  // Send joystick data at 20Hz (every 50ms)
  if (millis() - lastSerialTime > SERIAL_INTERVAL) {
    sendJoystickData();      // Send current joystick position
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
  
  // COMMAND: EMERGENCY - Emergency stop
  else if (command.equals("EMERGENCY")) {
    emergencyStop();
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