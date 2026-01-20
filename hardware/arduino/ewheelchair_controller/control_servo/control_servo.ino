#include <Servo.h>

// Déclaration des servos
Servo servoAvantArriere;  // Axe Y (avant/arrière)
Servo servoGaucheDroite;  // Axe X (gauche/droite)

// --- Broches ---
// Arduino
const int brocheVRx = A0;
const int brocheVRy = A1;
// ServoM
const int brocheServoAvantArriere = 9;
const int brocheServoGaucheDroite = 10;
// UltraSon
const int trigPin_one = 6;  // Capteur ultrasons 1
const int trigEcho_one = 7;  // Capteur ultrasons 1

const int trigPin_two = 46;  // Capteur ultrasons 2
const int trigEcho_two = 44;  // Capteur ultrasons 2

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
  Serial.begin(9600);

  servoAvantArriere.attach(brocheServoAvantArriere);
  servoGaucheDroite.attach(brocheServoGaucheDroite);

  // Initialisation à la position neutre
  servoAvantArriere.write(angleNeutreAvantArriere);
  servoGaucheDroite.write(angleNeutreGaucheDroite);
  //Serial.println("Système prêt. Déplacez le joystick.");
}

void loop() {

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

  // ---------
  digitalWrite(trigPin_one, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin_one, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_one, LOW);

  digitalWrite(trigPin_two, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin_two, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_two, LOW);

  long duree_one = pulseIn(trigEcho_one, HIGH, 30000);
  long duree_two = pulseIn(trigEcho_two, HIGH, 30000);

  if (duree_one > 0) {
    int distance_one = duree_one * 0.034 / 2;
    Serial.print("Distance : ");
    Serial.print(distance_one);
    Serial.println(" cm");
  } else {
    Serial.println("Mesure invalide");
  }
    if (duree_two > 0) {
    int distance_two = duree_two * 0.034 / 2;
    Serial.print("Distance : ");
    Serial.print(distance_two);
    Serial.println(" cm");
  } else {
    Serial.println("Mesure invalide");
  }
  //------------------

  // Envoi des commandes aux servos
  servoGaucheDroite.write(angleX);
  servoAvantArriere.write(angleY);

  // Affichage pour débogage
        //Serial.print("X: ");
  Serial.println("Joystick");
  Serial.println(angleX);
        //Serial.print(" (Delta: ");
        //Serial.print(map(deltaX, -512, 512, -amplitude, amplitude));
        //Serial.print(") | Y: ");
  Serial.println(angleY);
        //Serial.print(" (Delta: ");
        //Serial.print(map(deltaY, -512, 512, -amplitude, amplitude));
        //Serial.println(")");

  delay(300); // Délai pour une réponse réactive
}
