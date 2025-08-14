/*
 * acelerometer.h
 *
 *  Created on: Jun 6, 2025
 *      Author: mrxmi
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "stm32f4xx_hal.h"
#include "math.h"
//#include <stdint.h>


#define ACCEL_CS_LOW()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)  // Chip Select LOW
#define ACCEL_CS_HIGH()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)    // Chip Select HIGH


// Accelerometer
extern uint8_t reg;  // WHO_AM_I register (0x0F) with read bit (0x80)
extern uint8_t id;
extern uint8_t accReg[6];
extern int16_t accX;
extern int16_t accY;
extern int16_t accZ;
extern float accValues[3];
extern float accAngle;
extern uint8_t simpleNum;
extern float simpleAvgAngle[13];

extern uint8_t isEnabled;
extern uint8_t enableAcc[2];
extern uint8_t enableDRY[2];
extern uint8_t status;
extern uint8_t data[6];
extern uint8_t const range;


extern float angle;
extern float previousAngle;


void AccelInit(SPI_HandleTypeDef *hspi);


void readAccelerometer(SPI_HandleTypeDef *hspi);
float filterAngleMovingAverage();



#endif /* INC_ACCELEROMETER_H_ */
