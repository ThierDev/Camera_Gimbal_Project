#include "Kalman_Lib.h"


Kalman::Kalman() {
	Q_angle = 0.001f; // Erreur en position. 
	Q_error_s = 0.003f;// Erreur en vitesse. 
	R_measure = 0.03f;// Variance(v)

	angle_after_kalman = 0.0f;// remet à zéro l'angle
	speed_after_kalman = 0.0f; // remet à zéro l'erreur

	// Ceci est la matrice de covariance d'erreur// à t=0 , matrice==0
	P[0][0] = 0.0f; 
	P[0][1] = 0.0f;
	P[1][0] = 0.0f;
	P[1][1] = 0.0f;


};

float Kalman::getAngle(float newAngle, float newRate, float dt) {

	//Étape-1// on calcul l'angle avec (\theta avec dérive)
	speed_after_kalman_corrected = newRate - speed_after_kalman;
	angle_after_kalman += speed_after_kalman_corrected* dt; // ici on voit l'intégration

	//Étape-2 // Mise a jour de la matrice de covariance

	P[0][0] += dt*(dt*(P[1][1]) - P[1][0] - P[0][1] + Q_angle);
	P[0][1] -= dt*P[1][1];
	P[1][0] -= dt*P[1][1];
	P[1][1] += Q_error_s * dt;

	//Étape-4// Calcul du scalaire S 

	float S = P[0][0] + R_measure; // erreur totale (Estimation + Mesuré) 

	//Étape-5// Calcul du vecteur gain de kalman
	float K[2];
	K[0] = (P[0][0]) / S;
	K[1] = (P[0][1]) / S;

	//Étape-3// On calcule l'angle \theta et le \theta\dot dérive. 
	float y = newAngle - angle_after_kalman; // calcul de la variable d'innovation. 

	//Etape-6// on calcule l'angle avec une proportionnalite de kalman
	angle_after_kalman += K[0] * y;
	speed_after_kalman += K[1] * y;

	//Etape-7// calcul de la matrice de covariance a posteriori. 
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle_after_kalman; // on retourne l'angle que l'on veut. 

};
// pointeur qui permet de définir l'angle de départ. 
void Kalman::setAngle(float angle_after_kalman) { this->angle_after_kalman = angle_after_kalman; };

