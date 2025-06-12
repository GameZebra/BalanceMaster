/*
 * gyroscope.c
 *
 *  Created on: Jun 12, 2025
 *      Author: mrxmi
 */

#include "gyroscope.h"

uint16_t gyroValue = 0;
float angularVelocity = 0;
float measuredVoltage = 0;
float gyroAngle = 0;
float previousGyroAngle = 0;


void calculateGyroAngle(){
	measuredVoltage = (gyroValue / 4095.0)*2.9;
	angularVelocity = (measuredVoltage - 1.47)/0.01375/2; // degrees/ time
	if (angularVelocity > 0.7 || angularVelocity < -0.7){
		gyroAngle -= angularVelocity * 0.001; // 1/1000 s
		// - because the positive and negative sides of the Acc and Gyro are different
	}
}


