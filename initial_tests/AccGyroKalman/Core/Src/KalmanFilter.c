/*
 * KalmanFilter.c
 *
 *  Created on: Aug 20, 2025
 *      Author: mrxmi
 */

#include "KalmanFilter.h"

float32_t X_data[STATE_SIZE] = {0, 0, 0};
float32_t P_data[STATE_SIZE*STATE_SIZE] = {	1, 0, 0,	// covariance P initial guess
											0, 1, 0,
											0, 0, 1};
// process noise Q
float32_t Q_data[STATE_SIZE*STATE_SIZE] = {	0.15,		// q_theta
											0.23,		// q_omega
											0.000001};	// q_bias
//measurement noise R
float32_t R_data[MEAS_SIZE*MEAS_SIZE] = {	5,			//acc noise variance
										 	1.44};		//gyro noise variance

// measurments matrixes
float32_t H_data[MEAS_SIZE*STATE_SIZE] = {	1, 0, 0,	//H_acc
											0, 1, 1};	//H_gyro

// state transition matrix A
float32_t A_data[STATE_SIZE*STATE_SIZE] = {	1, gyroTd, 0,
											0, 1, 0,
											0, 0, 0};
// a priori
arm_matrix_instance_f32 X_pred;
float32_t X_pred_data[STATE_SIZE];
arm_matrix_instance_f32 AP, APAT;
float32_t AP_data[STATE_SIZE*STATE_SIZE], APAT_data[STATE_SIZE*STATE_SIZE];

arm_matrix_instance_f32 AT;
float32_t AT_data[STATE_SIZE*STATE_SIZE];


// K = P*H' * inv(H*P*H' + R)
arm_matrix_instance_f32 HT, HP, HPHT, HPHT_plus_R, HPHT_plus_R_inv;
float32_t HT_data[STATE_SIZE*MEAS_SIZE], HP_data[STATE_SIZE*MEAS_SIZE];
float32_t HPHT_data[MEAS_SIZE*MEAS_SIZE], HPHT_plus_R_data[MEAS_SIZE*MEAS_SIZE];
float32_t HPHT_plus_R_inv_data[MEAS_SIZE*MEAS_SIZE];

// X = X_pred + K*(z - H*X_pred)
float32_t z_data[MEAS_SIZE] = {0, 0};
arm_matrix_instance_f32 z;


arm_matrix_instance_f32 HX, z_minus_HX, Kz;
float32_t HX_data[MEAS_SIZE], z_minus_HX_data[MEAS_SIZE], Kz_data[STATE_SIZE];


void KalmanInit(){
	arm_mat_init_f32(&X, STATE_SIZE, 1, X_data);
	arm_mat_init_f32(&P, STATE_SIZE, STATE_SIZE, P_data);
	arm_mat_init_f32(&Q, STATE_SIZE, STATE_SIZE, Q_data);
	arm_mat_init_f32(&R, MEAS_SIZE, MEAS_SIZE, R_data);
	arm_mat_init_f32(&H, MEAS_SIZE, STATE_SIZE, H_data);


	// X_pred = A * X
	arm_mat_init_f32(&A, STATE_SIZE, STATE_SIZE, A_data);
	arm_mat_init_f32(&X_pred, STATE_SIZE, 1, X_pred_data);



	// P_pred = A*P*A' + Q
	arm_mat_init_f32(&AP, STATE_SIZE, STATE_SIZE, AP_data);
	arm_mat_init_f32(&APAT, STATE_SIZE, STATE_SIZE, APAT_data);

	arm_mat_init_f32(&AT, STATE_SIZE, STATE_SIZE, AT_data);

	// K = P*H' * inv(H*P*H' + R)
	arm_mat_init_f32(&HT, STATE_SIZE, MEAS_SIZE, HT_data);


	arm_mat_init_f32(&z, MEAS_SIZE, 1, z_data);

	arm_mat_init_f32(&HX, MEAS_SIZE, 1, HX_data);
	arm_mat_init_f32(&z_minus_HX, MEAS_SIZE, 1, z_minus_HX_data);
	arm_mat_init_f32(&Kz, STATE_SIZE, 1, Kz_data);


}


void KalmanPredict(){
	// X_pred = A * X
	arm_mat_mult_f32(&A, &X, &X_pred);

	// P_pred = A*P*A' + Q
	arm_mat_mult_f32(&A, &P, &AP);

	arm_mat_trans_f32(&A, &AT);
	arm_mat_mult_f32(&AP, &AT, &APAT);
	arm_mat_add_f32(&APAT, &Q, &P);  // P = P_pred
}

void KalmanUpdate(){
	// TODO leave here only the calculations
	// move the initializations in the init
	// check the where the measurments enter and leave them here




	// K = P*H' * inv(H*P*H' + R)
	arm_mat_trans_f32(&H, &HT);
	arm_mat_mult_f32(&P, &HT, &HP);
	arm_mat_mult_f32(&H, &HP, &HPHT);
	arm_mat_add_f32(&HPHT, &R, &HPHT_plus_R);

	// Since measurement is 1x1, inverse is simple:
	HPHT_plus_R_inv_data[0] = 1.0f / HPHT_plus_R_data[0];
	arm_mat_init_f32(&HPHT_plus_R_inv, MEAS_SIZE, MEAS_SIZE, HPHT_plus_R_inv_data);

	arm_mat_mult_f32(&HP, &HPHT_plus_R_inv, &K);

	// X = X_pred + K*(z - H*X_pred)
	z_data[0] = accAngle;
	z_data[1] = angularVelocity;

	arm_mat_mult_f32(&H, &X_pred, &HX);
	arm_mat_sub_f32(&z, &HX, &z_minus_HX);
	arm_mat_mult_f32(&K, &z_minus_HX, &Kz);
	arm_mat_add_f32(&X_pred, &Kz, &X);

}






