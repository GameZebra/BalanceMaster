/*
 * accelerometer.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mrxmi
 */

#include "accelerometer.h"


uint8_t reg = 0x0F | 0x80;  // WHO_AM_I register (0x0F) with read bit (0x80)
uint8_t id = 0x0;
uint8_t accReg [6] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
float accValues[3];
float accAngle = 0;
uint8_t simpleNum = 13;
float simpleAvgAngle[13];

uint8_t isEnabled = 0;
uint8_t enableAcc[2] = {0x20, 0x67};
uint8_t enableDRY[2] = {0x23, 0xC8};
uint8_t status = 0;
uint8_t data[6];
uint8_t const range = 2;


float angle = 0;
float previousAngle = 0;

void AccelInit(SPI_HandleTypeDef *hspi){
	  uint8_t isEnabled = 0;
	  uint8_t enableAcc[2] = {0x20, 0x67};
	  uint8_t enableDRY[2] = {0x23, 0xC8};



	  uint8_t msg = (enableAcc[0] | 0x80);
	  ACCEL_CS_LOW();
	  msg = (enableAcc[0] & 0x3F);
	  HAL_SPI_Transmit(hspi, &msg, 1, HAL_MAX_DELAY);
	  HAL_SPI_Transmit(hspi, &enableAcc[1], 1, HAL_MAX_DELAY);
	  ACCEL_CS_HIGH();
	  HAL_Delay(10);

	  msg = (enableAcc[0] | 0x80);
	  ACCEL_CS_LOW();
	  HAL_SPI_Transmit(hspi, &msg, 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(hspi, &isEnabled, 1, HAL_MAX_DELAY);
	  ACCEL_CS_HIGH();
	  HAL_Delay(10);


	  msg = enableDRY[0] & 0x3F;
	  ACCEL_CS_LOW();
	  HAL_SPI_Transmit(hspi, &msg, 1, HAL_MAX_DELAY);
	  HAL_SPI_Transmit(hspi, &enableDRY[1], 1, HAL_MAX_DELAY);
	  ACCEL_CS_HIGH();                 // Disable
	  HAL_Delay(10);
}


void readAccelerometer(SPI_HandleTypeDef *hspi){
	uint8_t msg;
	for(short i = 0; i<6; i++){
		msg = accReg[i] | 0x80;
		ACCEL_CS_LOW();                  // Enable SPI communication
		HAL_SPI_Transmit(hspi, &msg, 1, HAL_MAX_DELAY);  // Send register address
		HAL_SPI_Receive(hspi, &data[i], 1, HAL_MAX_DELAY);    // Receive the ID
		ACCEL_CS_HIGH();                 // Disable SPI

	}
	accX = (data[0] | (data[1] << 8));
	accY = (data[2] | (data[3] << 8));
	accZ = (data[4] | (data[5] << 8));

	accValues[0] = (accX / 32767.0) * 2;
	accValues[1] = (accY / 32767.0) * 2;
	accValues[2] = (accZ / 32767.0) * 2;

	accAngle = atan(accValues[0]/accValues[2])*180/M_PI;

	// moving average
//	angle = filterAngleMovingAverage();

	if(fabs(angle) > 90.0f){
		angle = previousAngle;
	}
}

// for filtration (or not)
float filterAngleMovingAverage(){
	float angle = 0.0f;
	for(int i = 0; i< simpleNum; i++){
	  simpleAvgAngle[i] = simpleAvgAngle[i+1];
	}
	simpleAvgAngle[simpleNum-1] = accAngle;
	angle =0;
	for(int i = 0; i< simpleNum; i++){
	  angle += simpleAvgAngle[i];
	}
	angle /= simpleNum;
	return angle;
}

// Low Pass Filter

// Kalman Filter






