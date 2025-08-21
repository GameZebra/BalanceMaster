/*https://chatgpt.com/c/6861cf8e-fe10-8007-a8c0-86962881bd45
 * gyroscope.h
 *
 *  Created on: Jun 12, 2025
 *      Author: mrxmi
 */

#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_


#include "stm32f4xx_hal.h"
#include "math.h"


// Gyro
extern uint16_t gyroValue;
extern float angularVelocity;
extern float previousAngularVelocity;
extern float measuredVoltage;
extern float gyroAngle;
extern float previousGyroAngle;
#define gyroTd 0.01
extern float gyroAVelocityBias;
extern float gyroConstant;

// debug
extern float maxDiviat;


void calculateGyroAngle(); // before using there should be ADC reading of the sensor
void calculateGyroAVelocityBase(uint8_t averageOrder);

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);


#endif /* GYROSCOPE_H_ */
