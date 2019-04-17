#include<Wire.h>
#include <Servo.h>
#include<EEPROM.h>
#include "Kalman_Lib.h"

//Création des instances de Kalman// 
Kalman kalmanX ;
Kalman kalmanY ;
Kalman kalmanZ;

//Création des instances de SERVO
Servo M_Yaw;
Servo M_Roll;
Servo M_Pitch;

// ############## All the MPU 6050 STUFF ###################
const int gyro_address = 0x68;
int iteratif_1, iteratif_2;

#define LED_PIN 13 // (Arduino is 13)
bool blinkState = false;

float gyro_roll_cal; 
float gyro_pitch_cal;                                      
float gyro_yaw_cal;
float acc_x_cal,acc_y_cal,acc_z_cal;    

float gyro_roll_d , gyro_pitch_d , gyro_yaw_d;
float gyro_roll_d_cal,gyro_pitch_d_cal,gyro_yaw_d_cal;
float gyro_roll_raw, gyro_pitch_raw,gyro_yaw_raw ;
float acc_x_raw,acc_y_raw,acc_z_raw, temperature ;

float yaw_angle, pitch_angle,roll_angle ; 
float pitch_angle_kal,roll_angle_kal, yaw_angle_kal ; 
float pitch_level_adjust,roll_level_adjust,yaw_level_adjust ; 


//################################# gains du PID #######################################

//gains pour le roulis:
float P_gain_roll = 1.0;
float I_gain_roll = 0.0;
float D_gain_roll = 0;
int max_roll = 400;

//gains pour le tangage:
float P_gain_pitch = 1.0;
float I_gain_pitch = 0.0;     
float D_gain_pitch = 0;
int max_pitch = 400;

//gains pour le lacet:
float P_gain_yaw = 1.0;
float I_gain_yaw = 0.0;
float D_gain_yaw = 0.0;
int max_yaw = 400;


//################################# variables du PID #######################################

float roll_erreur, pitch_erreur, yaw_erreur;
//valeurs du PID pour le roulis:
float val_P_roll;
float val_I_roll;
float val_D_roll;
float val_I_roll_pcdt = 0.0;
float roll_erreur_pcdt = 0.0;
float val_correction_roll;
float M_roll_PID ; 


//valeurs du PID pour le tangage:
float val_P_pitch;
float val_I_pitch;
float val_D_pitch;
float val_I_pitch_pcdt = 0.0;
float pitch_erreur_pcdt = 0.0;
float val_correction_pitch;
float M_pitch_PID ;  

//valeurs du PID pour le lacet:
float val_P_yaw;
float val_I_yaw;
float val_D_yaw;
float val_I_yaw_pcdt = 0.0;
float yaw_erreur_pcdt = 0.0;
float val_correction_yaw;
float M_yaw_PID ; 

//############################## Variables EEPROM mémoire #################################
int address = 1;
int address1 = 2 ;
int address2,address3,address4,address5,address6;

int nbr_buzz, duree_buzz, interval_buzz, i ; 


int battery_voltage; 
int joy_roll_raw,joy_pitch_raw,joy_yaw_raw;
int joy_roll_pulse,joy_pitch_pulse,joy_yaw_pulse;
int joy_roll_pid, joy_pitch_pid, joy_yaw_pid ; 
int  commandex;
bool button_1 = true ;

//variables de temps 
unsigned long timer;
double dt; 

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup(){
    Wire.begin();
    /// Vérification de la bonne fréquence I2C 
    #if ARDUINO >= 157
      Wire.setClock(400000UL); // Set I2C frequency to 400kHz
    #else
      TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    #endif
    
    gyro_init(); // initialise le MPU6050 
    magn_init();// initialise le Magnétomètre. 
    Serial.begin(9600);
    pinMode(13, OUTPUT);/// configuration de la led informative
    
    M_Yaw.attach(8); // le fil sur l'arduino est blanc 
    M_Pitch.attach(9);// le fil sur l'arduino est bleu
    M_Roll.attach(10);// le fil sur l'arduino est jaune
    
    delay(50);
  // Condition  de démarage de la calibration : 
    buzzer_je_buzz(2,50,30);

    commandex = 1500; 
    
    joy_roll_raw = analogRead(0);
    delay(50);
    

    if (EEPROM.read(address) != 1)calibration_gyro();
    else if (joy_roll_raw > 990 && EEPROM.read(address) == 1) calibration_gyro();
    else{
      gyro_roll_cal = EEPROM.read(address1);
      gyro_pitch_cal = EEPROM.read(address2);
      gyro_yaw_cal = EEPROM.read(address3);
      acc_x_cal = EEPROM.read(address4);
      acc_y_cal = EEPROM.read(address5);
      acc_z_cal = EEPROM.read(address6);
     } 
   
     //https://en.wikipedia.org/wiki/Atan2 pour l'explication des opérateurs utilisés pour la suite
    gyro_signal();
    magn_signal();
    double roll_angle  = atan2(acc_y_raw,acc_z_raw) * 57.2951; 
    double pitch_angle = atan(-acc_x_raw / sqrt(acc_y_raw* acc_y_raw + acc_z_raw * acc_z_raw)) * 57.2951;
    kalmanX.setAngle(roll_angle); // définition des angles de départ
    kalmanY.setAngle(pitch_angle);
    kalmanZ.setAngle(yaw_angle);
    
  
    timer = micros(); //Retourne le nombre de microsecondes écoulées depuis que l'arduino a commencé à lire le code    

  //notre batterie est à 12,6V max, ce qui correspond à 1023 en analogRead(0) 
  battery_voltage = (analogRead(0) + 65)*1.2317; // Pour établir une proportionnalité entre la valeur lue avec analogRead et la tension en Volts.
      
}

void loop() {
  double dt = (double)(micros() - timer) / 1000000; // Calcul de delta t
  timer = micros();
  gyro_signal();

  gyro_roll_d = rad_to_deg(gyro_roll_raw);/// converti en deg/sec les valeurs non calibrer. 
  gyro_pitch_d = rad_to_deg(gyro_pitch_raw);
  gyro_yaw_d = rad_to_deg(gyro_yaw_raw) ;

  gyro_roll_d_cal = rad_to_deg(gyro_roll_raw); /// convertir les valeurs calibrer en deg/sec
  gyro_pitch_d_cal = rad_to_deg(gyro_pitch_raw);
  gyro_yaw_d_cal = rad_to_deg(gyro_yaw_raw) ;

  
  double roll_angle  = atan2(acc_y_raw, acc_z_raw) * 57.2951; //on calcule la valeur de l'angle en fonction d'informations donnée en repère cartésien
  double pitch_angle = atan(-acc_x_raw / sqrt(acc_y_raw* acc_y_raw + acc_z_raw * acc_z_raw)) * 57.2951;
  // Ceci résout le problème de la transition de l'angle de l'accéléromètre entre -pi et pi 
  if ((roll_angle < -90 && roll_angle_kal > 90) || (roll_angle > 90 && roll_angle_kal < -90)) {
    kalmanX.setAngle(roll_angle);
    roll_angle_kal = roll_angle;
    }
  else    roll_angle_kal = kalmanX.getAngle(roll_angle, gyro_roll_d, dt); // Calculate the angle using a Kalman filter

  if (abs(roll_angle_kal) > 90)gyro_pitch_d = -gyro_pitch_d ; // Inverse le signal pour avoir rester dans l'interval -\pi/2 \pi/2 
  pitch_angle_kal = kalmanY.getAngle(pitch_angle,gyro_pitch_d, dt);
  roll_angle_kal = kalmanX.getAngle(roll_angle, gyro_roll_d, dt);
  yaw_angle_kal = kalmanZ.getAngle(yaw_angle,gyro_yaw_d, dt) ; 

/// Détermination des correction angulaire à apporter sur les moteurs. On refait la proportion 180 degre vers des pulsation 1000-1500 et 1500-2000 en multipliant par 15. 
  roll_level_adjust = roll_angle_kal*15 ;
  pitch_level_adjust = pitch_angle_kal*15 ;
  yaw_level_adjust = yaw_angle*15 ; 
  
  val_I_roll = 0;
  roll_erreur_pcdt = 0;
  val_I_pitch=0;
  pitch_erreur_pcdt =0;
  val_I_yaw = 0;
  yaw_erreur_pcdt= 0;

   PID();
   
  int start=2;
  int throttle=1500;
  if (start==2){
    
    M_yaw_PID = throttle + val_correction_yaw;
    M_roll_PID = throttle + val_correction_roll;
    M_pitch_PID = throttle + val_correction_pitch ;
// limite la valeur maximal pour ne pas décalibrer les ESC 
    if(M_yaw_PID > 2000)M_yaw_PID=2000;
    if(M_roll_PID > 2000)M_roll_PID=2000;
    if(M_pitch_PID > 2000)M_pitch_PID=2000;
// Limite la valeur minimal pour ne pas décalibrer les ESC 
    if(M_yaw_PID <1000)M_yaw_PID=1000;
    if(M_roll_PID <1000)M_roll_PID=1000;
    if(M_pitch_PID <1000)M_pitch_PID=1000;
    
    
  
  }
  else{
    // Si tous va mal , on envoi la pulsation médiane dans ESC. 
    M_yaw_PID = throttle;
    M_roll_PID = throttle;
    M_pitch_PID = throttle;
    
  }
    
  // Envoie les corrections au moteurs.  
   M_Yaw.writeMicroseconds(M_yaw_PID);
   M_Roll.writeMicroseconds(M_roll_PID);
   M_Pitch.writeMicroseconds(M_pitch_PID);


   ///////////////////////// Printing section for essential values //////////////////////////

   Serial.print("  PID tangage:");Serial.print(val_correction_pitch);Serial.print("  PID roulis:");Serial.print(val_correction_roll);Serial.print("  PID lacet:");Serial.println(val_correction_yaw);
   //Serial.print("Roll  :"),Serial.print(gyro_roll_d),Serial.print("Pitch  :"),Serial.print(gyro_pitch_d),Serial.print("Yaw  :"),Serial.println(gyro_yaw_d);
   //Serial.print("Roll_angle_K  :"),Serial.print(roll_angle_kal),Serial.print("Pitch_angle_k  :"),Serial.print(pitch_angle_kal),Serial.print("Yaw_angle  :"),Serial.println(yaw_angle_kal);
   //Serial.print("Roll_angle:  "),Serial.print(roll_angle),Serial.print("Pitch_angle:  "),Serial.print(pitch_angle),Serial.print("Yaw_angle  :"),Serial.println(yaw_angle);
   //Serial.print("Roll_joy:  "),Serial.print(joy_roll_pid),Serial.print("Pitch_Joy:  "),Serial.println(joy_pitch_pid);
}

// Fonction qui converti en deg/s 
float rad_to_deg(float angle){
  float angle_c =((angle_c)*0.7 + (angle/57.14286)*0.3);
  return angle_c ; 
}

void PID()
{
  //calcul des différentes erreurs:
  roll_erreur = gyro_roll_d_cal;//
  pitch_erreur = gyro_pitch_cal;// 
  yaw_erreur = gyro_yaw_d_cal;// 


  //calcul de la correction à apporter au roulis:
  val_P_roll = roll_erreur*P_gain_roll;
  val_I_roll = val_I_roll_pcdt + roll_erreur*I_gain_roll + roll_level_adjust;
  val_D_roll = (roll_erreur - roll_erreur_pcdt)*D_gain_roll;

  if(val_I_roll > max_roll)
  {
    val_I_roll = max_roll;
  }
  else if(val_I_roll < max_roll * -1)
  {
    val_I_roll = max_roll * -1;
  }

  val_correction_roll = val_P_roll + val_I_roll + val_D_roll;
  if (val_correction_roll > max_roll)
  {
    val_correction_roll = max_roll;
  }
  else if(val_correction_roll < -1*max_roll)
  {
    val_correction_roll = -1*max_roll;
  }

  val_I_roll_pcdt = val_I_roll;
  roll_erreur_pcdt = roll_erreur;

  //calcul de la correction à apporter au tangage:
  val_P_pitch = pitch_erreur*P_gain_pitch;
  val_I_pitch = val_I_pitch_pcdt + pitch_erreur*I_gain_pitch + pitch_level_adjust;
  val_D_pitch = (pitch_erreur - pitch_erreur_pcdt)*D_gain_pitch;

  if(val_I_pitch > max_pitch)
  {
    val_I_pitch = max_pitch;
  }
  else if(val_I_pitch < max_pitch * -1)
  {
    val_I_pitch = max_pitch * -1;
  }

  val_correction_pitch = val_P_pitch + val_I_pitch + val_D_pitch;
  if (val_correction_pitch > max_pitch)
  {
    val_correction_pitch = max_pitch;
  }
  else if(val_correction_pitch < -1*max_pitch)
  {
    val_correction_pitch = -1*max_pitch;
  }
  
  val_I_pitch_pcdt = val_I_pitch;
  pitch_erreur_pcdt = pitch_erreur;
  
  //calcul de la correction à apporter au lacet:
  val_P_yaw = yaw_erreur*P_gain_yaw;
  val_I_yaw = val_I_yaw_pcdt + yaw_erreur*I_gain_yaw + yaw_level_adjust;
  val_D_yaw = (yaw_erreur - yaw_erreur_pcdt)*D_gain_yaw;

  if(val_I_yaw > max_yaw)
  {
    val_I_yaw = max_yaw;
  }
  else if(val_I_yaw < max_yaw * -1)
  {
    val_I_yaw = max_yaw * -1;
  }

  val_correction_yaw = val_P_yaw + val_I_yaw + val_D_yaw;
  if (val_correction_yaw > max_yaw)
  {
    val_correction_yaw = max_yaw;
  }
  else if(val_correction_yaw < -1*max_yaw)
  {
    val_correction_yaw = -1*max_yaw;
  }

  val_I_yaw_pcdt = val_I_yaw;
  yaw_erreur_pcdt = yaw_erreur;
}
// ########################### Fonction simple du gyro ####################################
void gyro_init(){
    Wire.begin();
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x00);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro
}
void magn_init(){
  Wire.beginTransmission(0x1E); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
}
void magn_signal(){
  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(0x1E);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  int magn_x,magn_y,magn_z ;
  Wire.requestFrom(0x1E, 6);
  if(6<=Wire.available()){
    magn_x = Wire.read()<<8|Wire.read();
    magn_z = Wire.read()<<8|Wire.read();
    magn_y = Wire.read()<<8|Wire.read();
  }
  float heading =0.8*heading + 0.2*atan2(magn_y,magn_x); 
  float yaw_angle = heading * 180/3.14159265358979323846264338327950288;  converti en deg 
  
}
void calibration_gyro(){
    digitalWrite(13,HIGH);
    delay(5);
      //Calibration 
    for (iteratif_1 = 0; iteratif_1 < 3000 ; iteratif_1 ++){              //Accumule 3000 valeurs pour la calibration
      if(iteratif_1 % 15 == 0)digitalWrite(13, !digitalRead(13));  //Fait clignoter la LED pour indiquer la calibration.
      gyro_signal();
      //Appelle la fonction gyroscope 
      gyro_roll_cal +=gyro_roll_raw;                               // Fait la somme sur 3000 valeurs sur tous les axes 
      gyro_pitch_cal +=gyro_pitch_raw;                              
      gyro_yaw_cal += gyro_yaw_raw;
      acc_x_cal +=acc_x_raw;
      acc_y_cal +=acc_y_raw;
      acc_z_cal +=acc_z_raw;
      M_Yaw.writeMicroseconds(400);
      M_Roll.writeMicroseconds(400);
      M_Pitch.writeMicroseconds(400);
                                       
      //pour éviter une décalibration des ESC on envoi encore une pulsation de 1000us 
      delay(1);                                              
      } 
      //EEPROM.put(address,int(1));// met le bouléen mémoire a 1   
          //Maintenant on a optenu 3000 valeurs on peut faire la moyenne 
      gyro_roll_cal /= 3000;                                       //Divise le total par 3000 
      gyro_pitch_cal /= 3000;                                      
      gyro_yaw_cal /= 3000;
      acc_x_cal /=3000;
      acc_y_cal/= 3000;
      acc_z_cal /= 3000;   
      EEPROM.put(address1,gyro_roll_cal);
      address2=address1+sizeof(float(gyro_roll_cal));
      EEPROM.put(address2,gyro_pitch_cal);
      address3=address2+sizeof(float(gyro_pitch_cal));
      EEPROM.put(address3,gyro_yaw_cal);
      address4=address3+sizeof(float(gyro_yaw_cal));
      EEPROM.put(address4,acc_x_cal);
      address5=address4+sizeof(float(acc_x_cal));
      EEPROM.put(address5,acc_y_cal);
      address6=address5+sizeof(float(acc_y_cal));
      EEPROM.put(address6,acc_z_cal); */
      
      digitalWrite(13,LOW);
      Serial.println("Calibration gyro sucess"); 
      Serial.print(gyro_roll_cal),Serial.print(gyro_pitch_cal),Serial.println(gyro_yaw_cal);

}

void gyro_signal()
{
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_x_raw = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_x variable.
    acc_y_raw = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_y variable.
    acc_z_raw = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_roll_raw = Wire.read()<<8|Wire.read();                                  //Read high and low part of the angular data.
    gyro_pitch_raw = Wire.read()<<8|Wire.read();                                  //Read high and low part of the angular data.
    gyro_yaw_raw = Wire.read()<<8|Wire.read();                                  //Read high and low part of the angular data.

    if(iteratif_1 == 3000){
      float acc_x_raw_cal=acc_x_raw - acc_x_cal;
      float acc_y_raw_cal= acc_y_raw - acc_y_cal;
      float acc_z_raw_cal= acc_z_raw_cal - acc_z_cal;

      float gyro_roll_raw_cal = gyro_roll_raw - gyro_roll_cal;
      float gyro_pitch_raw_cal = gyro_pitch_raw - gyro_pitch_cal; 
      float gyro_yaw_raw_cal =gyro_yaw_raw - gyro_yaw_cal;  
    } 
 }
// Fonction buzzer prend en argument : (nbr de buzz , longueur du buzz, temps entre buzz ) 
int buzzer_je_buzz(int(nbr_buzz),int(duree_buzz),int(interval_buzz)){
  i=1;
  while(i<= nbr_buzz){
    digitalWrite(13,HIGH);
    delay(duree_buzz);
    digitalWrite(13,LOW);
    delay(interval_buzz);
    i+=1;
  }
}

int Universal_joystick(int(pulse)){
    if(pulse <470){
    if(commandex > 1100){
      commandex -= 2;
    }
    else(commandex = 1100);
  }
  else if(pulse >550){
    if(commandex < 1900){
      commandex += 2;
    }
    else(commandex = 1900);
  }
  
  return commandex ;
   
  
}
  
 

