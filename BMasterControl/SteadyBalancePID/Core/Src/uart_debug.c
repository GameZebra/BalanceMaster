/*
 * uart_debug.c
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#include "uart_debug.h"

void pcTransmitTime(UART_HandleTypeDef *uart5){
	//time stamp
	uint32_t tick = HAL_GetTick();
	HAL_UART_Transmit(uart5, &tick, 4, 10);
}

void pcTransmitAngles(UART_HandleTypeDef *uart5){
	// angles
	HAL_UART_Transmit(uart5, &gyroAngle, 4, 10);
	HAL_UART_Transmit(uart5, &accAngle, 4, 10);
	HAL_UART_Transmit(uart5, &angle, 4, 10);
	HAL_UART_Transmit(uart5, &setpoint, 4, 10);
}

void pcTransmitEncoders(UART_HandleTypeDef *uart5){
	//encoders
	HAL_UART_Transmit(uart5, &encoderL, 2, 10);
	HAL_UART_Transmit(uart5, &encoderR, 2, 10);
	HAL_UART_Transmit(uart5, &encoderLSpeed, 2, 10);
	HAL_UART_Transmit(uart5, &encoderRSpeed, 2, 10);
}

void pcTransmitControll(UART_HandleTypeDef *uart5){
	// control
	uint8_t zero = 0;
	HAL_UART_Transmit(uart5, &error, 4, 10);
	HAL_UART_Transmit(uart5, &zero, 1, 10);
	HAL_UART_Transmit(uart5, &speed, 1, 10);
	HAL_UART_Transmit(uart5, &Kp, 4, 10);
	HAL_UART_Transmit(uart5, &Ki, 4, 10);
	HAL_UART_Transmit(uart5, &Kd, 4, 10);
	HAL_UART_Transmit(uart5, &derivative, 4, 10);
}

void pcTransmitAcc(UART_HandleTypeDef *uart5){
	//accelerometer debug
	HAL_UART_Transmit(uart5, &accValues[0], 4, 10);
	HAL_UART_Transmit(uart5, &accValues[2], 4, 10);

}


void pcTransmitBin(UART_HandleTypeDef *uart5){
	pcTransmitTime(uart5);
	pcTransmitAngles(uart5);
	pcTransmitEncoders(uart5);
	pcTransmitControll(uart5);
	pcTransmitAcc(uart5);
}


