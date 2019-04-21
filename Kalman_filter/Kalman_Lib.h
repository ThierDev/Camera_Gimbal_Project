#ifndef _Kalman_Lib_h_
#define _Kalman_Lib_h_

class Kalman {
public:
	Kalman();
	float getAngle(float acc_angle,float gyro_speed, float dt);
	void setAngle(float angle);

private:
	float Q_angle; //erreur angle de l'acc�l�rom�tre
	float Q_error_s;// erreur vitesse angulaire gyroscope
	float R_measure;// erreur sur la mesure des erreurs 
	
	float angle_after_kalman;// donc l'angle th�ta final
	float speed_after_kalman;//vitesse kalman th�ta\dot  (pour la d�rive) 
	float speed_after_kalman_corrected;// Vitesse de kalman th�ta\dot (sans d�rive) 
	float P[2][2];// matrice de covariance \in\scriptM_2(\doubleR) 

};
	
#endif