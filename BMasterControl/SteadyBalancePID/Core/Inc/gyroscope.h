/*
 * gyroscope.h
 *
 *  Created on: Jun 12, 2025
 *      Author: mrxmi
 */

#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_


#include "stm32f4xx_hal.h"



// Gyro
extern uint16_t gyroValue;
extern float angularVelocity;
extern float measuredVoltage;
extern float gyroAngle;
extern float previousGyroAngle;



void calculateGyroAngle(); // before using there should be ADC reading of the sensor


#endif /* GYROSCOPE_H_ */
