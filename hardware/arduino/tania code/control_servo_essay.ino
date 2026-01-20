#include <Servo.h>

// Déclaration des servos
//Servo servoAvantArriere;  // Axe Y (avant/arrière)
//Servo servoGaucheDroite;  // Axe X (gauche/droite)

// --- Broches ---
// Arduino
const int brocheVRx = A0;
const int brocheVRy = A1;
// ServoM
const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;
// UltraSon
const int trigPin_one = 30;  // Capteur ultrasons 1
const int trigEcho_one = 31;  // Capteur ultrasons 1

const int trigPin_two = 32;  // Capteur ultrasons 2
const int trigEcho_two = 33;  // Capteur ultrasons 2

const int trigPin_thr = 34;  // Capteur ultrasons 2
const int trigEcho_thr = 35;  // Capteur ultrasons 2

long duree; // durée de l'echo 
int distance; // distance 

// Angles neutres (différents pour chaque servo)
const int angleNeutreAvantArriere = 85;
const int angleNeutreGaucheDroite = 90;

// Amplitude de déplacement (identique pour les deux servos)
const int amplitude = 15;  // ±5° autour de la position neutre

void setup() {
  pinMode(trigPin_one, OUTPUT); // Configuration du port du Trigger comme une SORTIE 
  pinMode(trigEcho_one, INPUT);

  pinMode(trigPin_two, OUTPUT); // Configuration du port du Trigger comme une SORTIE 
  pinMode(trigEcho_two, INPUT);

  pinMode(trigPin_thr, OUTPUT); // Configuration du port du Trigger comme une SORTIE 
  pinMode(trigEcho_thr, INPUT);
  Serial.begin(9600);

  servoAvantArriere.attach(brocheServoAvantArriere);
  servoGaucheDroite.attach(brocheServoGaucheDroite);

  // Initialisation à la position neutre
  servoAvantArriere.write(angleNeutreAvantArriere);
  servoGaucheDroite.write(angleNeutreGaucheDroite);
  Serial.println("Système prêt. Déplacez le joystick.");
}

void loop() {

  int d1 = readUltrasonic(trigPin_one, trigEcho_one);
  delay(60);

  int d2 = readUltrasonic(trigPin_two, trigEcho_two);
  delay(60);

  int d3 = readUltrasonic(trigPin_thr, trigEcho_thr);
  delay(60);

  Serial.print("D1: "); Serial.print(d1);
  Serial.print(" | D2: "); Serial.print(d2);
  Serial.print(" | D3: "); Serial.println(d3);


  // Lecture des valeurs du joystick (0-1023)
  int valeurX = analogRead(brocheVRx);
  int valeurY = analogRead(brocheVRy);

  // Mapping des valeurs du joystick (-512 à +512) vers (-amplitude à +amplitude)
  int deltaX = map(valeurX, 0, 1023, -512, 512);  // Centré sur 0
  int deltaY = map(valeurY, 0, 1023, -512, 512);  // Centré sur 0

  // Conversion en angles (avec contrainte d'amplitude)
  int angleX = angleNeutreGaucheDroite + map(deltaX, -512, 512, -amplitude, amplitude);
  int angleY = angleNeutreAvantArriere + map(deltaY, -512, 512, -amplitude, amplitude);

  // Limites de sécurité (redondantes ici, mais utiles pour éviter les erreurs)
  angleX = constrain(angleX, angleNeutreGaucheDroite - amplitude, angleNeutreGaucheDroite + amplitude);
  angleY = constrain(angleY, angleNeutreAvantArriere - amplitude, angleNeutreAvantArriere + amplitude);

  
  //------------------

  // Envoi des commandes aux servos
  servoGaucheDroite.write(angleX);
  servoAvantArriere.write(angleY);

  // Affichage pour débogage
        //Serial.print("X: ");
  //Serial.println("Joystick");
  //Serial.println(angleX);
        //Serial.print(" (Delta: ");
        //Serial.print(map(deltaX, -512, 512, -amplitude, amplitude));
        //Serial.print(") | Y: ");
  //Serial.println(angleY);
        //Serial.print(" (Delta: ");
        //Serial.print(map(deltaY, -512, 512, -amplitude, amplitude));
        //Serial.println(")");

  delay(500); // Délai pour une réponse réactive
}

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
