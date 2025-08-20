/*
 * KalmanFilter.h
 *
 *  Created on: Aug 19, 2025
 *      Author: mrxmi
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include "arm_math.h"   // CMSIS DSP core functions
#include "matrix_functions.h" // Matrix functions


#define STATE_SIZE 3
#define MEAS_SIZE 1

extern arm_matrix_instance_f32 X;    // state vector [theta; omega; bias]
extern arm_matrix_instance_f32 P;    // covariance
extern arm_matrix_instance_f32 Q;    // process noise
extern arm_matrix_instance_f32 R;    // measurement noise
extern arm_matrix_instance_f32 K;    // Kalman gain
extern arm_matrix_instance_f32 H;    // measurement matrix

extern float32_t X_data[STATE_SIZE];
extern float32_t P_data[STATE_SIZE*STATE_SIZE];
extern float32_t Q_data[STATE_SIZE*STATE_SIZE];
extern float32_t R_data[MEAS_SIZE*MEAS_SIZE];
extern float32_t H_data[MEAS_SIZE*STATE_SIZE]; // measuring angle only

// X_pred = A * X
extern arm_matrix_instance_f32 A;
extern arm_matrix_instance_f32 X_pred;


// P_pred = A*P*A' + Q
extern arm_matrix_instance_f32 AP, APAT;
extern arm_matrix_instance_f32 AT;

// K = P*H' * inv(H*P*H' + R)
extern arm_matrix_instance_f32 HT, HP, HPHT, HPHT_plus_R, HPHT_plus_R_inv;


// X = X_pred + K*(z - H*X_pred)
extern float32_t z_data[MEAS_SIZE];
extern arm_matrix_instance_f32 z;


extern arm_matrix_instance_f32 HX, z_minus_HX, Kz;
extern float32_t HX_data[MEAS_SIZE], z_minus_HX_data[MEAS_SIZE], Kz_data[STATE_SIZE];

#endif /* INC_KALMANFILTER_H_ */
