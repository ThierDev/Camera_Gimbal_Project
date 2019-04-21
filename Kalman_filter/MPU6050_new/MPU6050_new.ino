#include <Wire.h>
#include <HeaderKalman.h>

#define RESTICT_PITCH //restriction du roulis à 90° pour éviter le gimbal lock

Kalman kalmanX
Kalman kalmanY

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle, gyroZangle_yaw; // angle calculé avec uniquement le gyroscope
double kalAngleX, kalAngleY; // Angle calculé à l'aide du filtre de kalman

uint32_t timer;

void setup() {
  Serial.begin(115200);
  Wire.begin

  delay(100); // Attente du temps nécessaire pour les les capteurs se stabilisent


  
//https://en.wikipedia.org/wiki/Atan2 pour l'explication des opérateurs utilisés pour la suite

  double roll  = atan2(accY, accZ) * RAD_TO_DEG; //arctan(accY/accZ) auquel on ajoute + ou - pi ou rien en fct des signes de accY et accZ
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // définition des angles de départ
  kalmanY.setAngle(pitch);
  gyroXangle = roll; //définition des axes correspondant aux roulis et au tangage
  gyroYangle = pitch;
  gyroZangle_yaw = yaw;

  timer = micros(); //Retourne le nombre de microsecondes écoulées depuis que l'arduino a commencé à lire le code

  
}

void loop() {


  double dt = (double)(micros() - timer) / 1000000; // Calcul de delta t
  timer = micros();
  double roll  = atan2(accY, accZ) * RAD_TO_DEG; //on calcule la valeur de l'angle en fonction d'informations donnée en repère cartésien
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Conversion en deg/s
  double gyroYrate = gyroY / 131.0; 
  double gyroZrate_yaw = gyroZ/131.0;
#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) { //résout le problème de transition entre -180 et 180 degrés, même si ce cas ne devrait pas se présenter dans notre système
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // prendr l'inverse de la valeur de l'angle
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
    //Reset du gyro si il dérive trop
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  delay(2);
}
