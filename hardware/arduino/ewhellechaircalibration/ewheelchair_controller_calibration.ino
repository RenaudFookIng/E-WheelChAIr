#include <Servo.h>
#include <EEPROM.h>

// ============================
// Déclaration des servos
// ============================
Servo servoAvantArriere;
Servo servoGaucheDroite;

// ============================
// Broches joystick
// ============================
const int brocheVRx = A0;
const int brocheVRy = A1;

// ============================
// Servos
// ============================
const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;

// ============================
// Ultrasons
// ============================
const int trigPin_one = 30;  
const int trigEcho_one = 31;
const int trigPin_two = 32;  
const int trigEcho_two = 33;
const int trigPin_thr = 34;  
const int trigEcho_thr = 35;

// ============================
// Paramètres servo
// ============================
const int angleNeutreAvantArriere = 85;
const int angleNeutreGaucheDroite = 90;

// ============================
// Fréquence loop
// ============================
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 50;

// ============================
// Commandes servo ROS
// ============================
int servoX_angle = angleNeutreGaucheDroite;
int servoY_angle = angleNeutreAvantArriere;

// ============================
// Calibration joystick
// ============================
int joyMinX, joyMaxX, joyCenterX;
int joyMinY, joyMaxY, joyCenterY;

// EEPROM signature
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_MAGIC_VALUE = 123;

// EEPROM data start
const int EEPROM_DATA_ADDR = 10;

// ============================
// Setup
// ============================
void setup() {
  Serial.begin(115200);

  pinMode(trigPin_one, OUTPUT); pinMode(trigEcho_one, INPUT);
  pinMode(trigPin_two, OUTPUT); pinMode(trigEcho_two, INPUT);
  pinMode(trigPin_thr, OUTPUT); pinMode(trigEcho_thr, INPUT);

  servoAvantArriere.attach(brocheServoAvantArriere);
  servoGaucheDroite.attach(brocheServoGaucheDroite);

  servoAvantArriere.write(angleNeutreAvantArriere);
  servoGaucheDroite.write(angleNeutreGaucheDroite);

  // Charger calibration EEPROM
  if (!loadCalibrationFromEEPROM()) {
    Serial.println("NO_CALIBRATION_FOUND");
    Serial.println("CALIBRATION_START");
    calibrateJoystick();
    saveCalibrationToEEPROM();
  }

  Serial.println("READY");
}

// ============================
// Calibration joystick auto
// ============================
void calibrateJoystick() {
  unsigned long startTime = millis();

  joyMinX = 1023;
  joyMaxX = 0;
  joyMinY = 1023;
  joyMaxY = 0;

  while (millis() - startTime < 3000) {
    int x = analogRead(brocheVRx);
    int y = analogRead(brocheVRy);

    joyMinX = min(joyMinX, x);
    joyMaxX = max(joyMaxX, x);

    joyMinY = min(joyMinY, y);
    joyMaxY = max(joyMaxY, y);

    delay(5);
  }

  long sumX = 0;
  long sumY = 0;
  for (int i = 0; i < 50; i++) {
    sumX += analogRead(brocheVRx);
    sumY += analogRead(brocheVRy);
    delay(2);
  }

  joyCenterX = sumX / 50;
  joyCenterY = sumY / 50;

  Serial.print("CALIB_X,"); Serial.print(joyMinX); Serial.print(",");
  Serial.print(joyCenterX); Serial.print(",");
  Serial.println(joyMaxX);

  Serial.print("CALIB_Y,"); Serial.print(joyMinY); Serial.print(",");
  Serial.print(joyCenterY); Serial.print(",");
  Serial.println(joyMaxY);
}

// ============================
// Sauvegarde EEPROM
// ============================
void saveCalibrationToEEPROM() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);

  EEPROM.put(EEPROM_DATA_ADDR + 0, joyMinX);
  EEPROM.put(EEPROM_DATA_ADDR + 4, joyMaxX);
  EEPROM.put(EEPROM_DATA_ADDR + 8, joyCenterX);

  EEPROM.put(EEPROM_DATA_ADDR + 12, joyMinY);
  EEPROM.put(EEPROM_DATA_ADDR + 16, joyMaxY);
  EEPROM.put(EEPROM_DATA_ADDR + 20, joyCenterY);

  Serial.println("CALIBRATION_SAVED");
}

// ============================
// Lecture EEPROM
// ============================
bool loadCalibrationFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    return false;
  }

  EEPROM.get(EEPROM_DATA_ADDR + 0, joyMinX);
  EEPROM.get(EEPROM_DATA_ADDR + 4, joyMaxX);
  EEPROM.get(EEPROM_DATA_ADDR + 8, joyCenterX);

  EEPROM.get(EEPROM_DATA_ADDR + 12, joyMinY);
  EEPROM.get(EEPROM_DATA_ADDR + 16, joyMaxY);
  EEPROM.get(EEPROM_DATA_ADDR + 20, joyCenterY);

  Serial.println("CALIBRATION_LOADED");
  return true;
}

// ============================
// Ultrasons
// ============================
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 30000);
  if (duree == 0) return -1;

  return duree * 0.034 / 2;
}

// ===================================
// ROS Serial Input Servo Command
// ===================================
void handleIncomingSerial() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("S,")) {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > 0) {
        String sx = line.substring(firstComma + 1, secondComma);
        String sy = (thirdComma > 0) ? line.substring(secondComma + 1, thirdComma)
                                     : line.substring(secondComma + 1);

        servoX_angle = sx.toInt();
        servoY_angle = sy.toInt();
      }

      if (thirdComma > 0) {
        String emergencyStr = line.substring(thirdComma + 1);
        if (emergencyStr.toInt() > 0) {
          servoX_angle = angleNeutreGaucheDroite;
          servoY_angle = angleNeutreAvantArriere;
        }
      }
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime < LOOP_INTERVAL_MS) return;
  lastLoopTime = currentTime;

  // ============================================================
  // 1) LECTURE DES CAPTEURS ULTRASONS
  // ============================================================
  // Arduino mesure les distances
  int d1 = readUltrasonic(trigPin_one, trigEcho_one);
  int d2 = readUltrasonic(trigPin_two, trigEcho_two);
  int d3 = readUltrasonic(trigPin_thr, trigEcho_thr);

  // ============================================================
  // 2) PUBLICATION ULTRASONS VERS ROS (CSV USB)
  // ============================================================
  // Format envoyé au Arduino Bridge Node :
  // "U,d1,d2,d3"
  Serial.print("U,");
  Serial.print(d1); Serial.print(",");
  Serial.print(d2); Serial.print(",");
  Serial.println(d3);

  // ============================================================
  // 3) LECTURE JOYSTICK (RAW + CALIBRATION)
  // ============================================================
  int rawX = analogRead(brocheVRx);
  int rawY = analogRead(brocheVRy);

  // Applique bornes calibration EEPROM
  rawX = constrain(rawX, joyMinX, joyMaxX);
  rawY = constrain(rawY, joyMinY, joyMaxY);

  // ============================================================
  // 4) PUBLICATION JOYSTICK VERS ROS (CSV USB)
  // ============================================================
  // Format envoyé au Arduino Bridge Node :
  // "J,rawX,rawY"
  // ROS décidera ensuite du mouvement
  Serial.print("J,");
  Serial.print(rawX, 6); 
  Serial.print(",");
  Serial.println(rawY, 6);

  // ============================================================
  // 5) APPLICATION DES COMMANDES SERVO (VENANT DE ROS)
  // ============================================================
  // IMPORTANT :
  // Arduino ne décide PAS du mouvement
  // Il applique UNIQUEMENT les angles reçus de ROS
  servoGaucheDroite.write(servoX_angle);
  servoAvantArriere.write(servoY_angle);

  // ============================================================
  // 6) RÉCEPTION DES COMMANDES ROS → ARDUINO (SERVO)
  // ============================================================
  // ROS envoie : "S,angleX,angleY,emergency"
  // Arduino lit et stocke servoX_angle / servoY_angle
  handleIncomingSerial();
}

