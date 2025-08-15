/*
 * gyroscope.c
 *
 *  Created on: Jun 12, 2025
 *      Author: mrxmi
 */

#include "gyroscope.h"

uint16_t gyroValue = 0;
float angularVelocity = 0;
float previousAngularVelocity = 0;
float measuredVoltage = 0;
float gyroAngle = 0;
float previousGyroAngle = 0;
float gyroTd = 0.01;
float gyroAVelocityBase = 1.465;
float gyroConstant = 0.00702176;
// debug
float maxDiviat = 0;



void calculateGyroAngle(){
	measuredVoltage = (gyroValue / 4095.0)*2.89;
	angularVelocity = (measuredVoltage - gyroAVelocityBase)/gyroConstant; // [degrees/ time]
//	if (fabs(angularVelocity) > 2.2){
//		gyroAngle -= angularVelocity * gyroTd; // 1/1000 s
//		// - because the positive and negative sides of the Acc and Gyro are different
//	}
//	else {
//		angularVelocity = 0;
//	}
	previousAngularVelocity = angularVelocity;
	if(fabs(angularVelocity) > maxDiviat){
		maxDiviat = fabs(angularVelocity);
	}
}

void calculateGyroAVelocityBase(uint8_t averageOrder){
	float temp = measuredVoltage;
	float sum = 0;

	for (uint8_t i = 0; i<averageOrder; i++){
		while(temp == measuredVoltage){
			// wait for the measured voltage to change
		}
		temp = measuredVoltage;
		sum += temp;
	}
	sum /= averageOrder;
	gyroAVelocityBase = sum;

}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hadc);
//  /* NOTE : This function Should not be modified, when the callback is needed,
//            the HAL_ADC_ConvCpltCallback could be implemented in the user file
//   */
//
//  gyroValue = HAL_ADC_GetValue(hadc);
//  calculateGyroAngle();
//
//}

