/*
 * qik_2s12v10_lib.c
 *
 *  Created on: Jun 5, 2025
 *      Author: mrxmi
 */
#include "qik_2s12v10_lib.h"


uint8_t const motor0[3] = {m0Forward, m0Brake, m0Reverse};	// the left motor
uint8_t const motor1[3] = {m1Reverese, m1Brake, m1Forward}; // the right motor
uint8_t rotation = 0;
uint8_t rotationOld = 0;
uint8_t speed = 0;
uint8_t brake = 127;


void MotorsBrake(UART_HandleTypeDef *huart, uint8_t *brake){
	HAL_UART_Transmit(huart, &motor0[1], 1, 20);
	HAL_UART_Transmit(huart, brake, 1, 20);
	HAL_UART_Transmit(huart, &motor1[1], 1, 20);
	HAL_UART_Transmit(huart, brake, 1, 20);
}

void MotorsOn(UART_HandleTypeDef *huart, uint8_t *speed, uint8_t rotation){
	HAL_UART_Transmit(huart, &motor0[rotation], 1, 20);
	HAL_UART_Transmit(huart, speed, 1, 20);
	// HAL_Delay(10);
	HAL_UART_Transmit(huart, &motor1[rotation], 1, 20);
	HAL_UART_Transmit(huart, speed, 1, 20);
	//HAL_Delay(10);
}
