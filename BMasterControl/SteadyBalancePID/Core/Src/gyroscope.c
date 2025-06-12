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
float gyroTd = 0.001;
float gyroAVelocityBias = 1.476;
float gyroConstant = 0.00702176;
// debug
float maxDiviat = 0;


void calculateGyroAngle(){
	measuredVoltage = (gyroValue / 4095.0)*2.9;
	angularVelocity = (measuredVoltage - gyroAVelocityBias)/gyroConstant; // degrees/ time
	if (fabs(angularVelocity) > 1.2){
		gyroAngle -= angularVelocity * gyroTd; // 1/1000 s
		// - because the positive and negative sides of the Acc and Gyro are different
	}
	if(fabs(angularVelocity) > maxDiviat){
		maxDiviat = fabs(angularVelocity);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  gyroValue = HAL_ADC_GetValue(hadc);
  calculateGyroAngle();

}

