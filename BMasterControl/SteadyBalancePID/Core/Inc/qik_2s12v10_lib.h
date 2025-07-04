/*
 * qik_2s12v10_lib.h
 *
 *  Created on: Feb 3, 2025
 *      Author: mrxmi
 */

#ifndef qik_2s12v10_lib
#define qik_2s12v10_lib

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "control.h"
#include "encoders.h"

#define getFirmwareVersion 	0x81
#define getErrorByte 		0x82
	// 0 Motor 0 Fault
	// 1 Motor 1 Fault
 	// 2 Motor 0 Over Current
	// 3 Motor 1 Over Current
	// 4 Serial Hardware Error
	// 5 CRC Error
	// 6 Format Error
	// 7 Timeout
#define getConfigParameter 0x83
#define setConfigParameter 0x84
	// syntax
	// 0x83, [parameter number], [parameter value], 0x55, 0x2A
	// the last 2 bytes are accidental protection

	// parameters:
	// 0 Device ID
	// 1 PWM Parameter
	// 2 Shut Down on Error
	// 3 Serial Timeout
	// 4 Motor M0 Acceleration
	// 5 Motor M1 Acceleration
	// 6 Motor M0 Brake Duration
	// 7 Motor M1 Brake Duration
	// 8 Motor M0 Current Limit / 2
	// 9 Motor M1 Current Limit / 2
	// 10 Motor M0 Current-Limit Response
	// 11 Motor M1 Current-Limit Response


	// returns:
	// 0 Command OK
	// 1 Bad Parameter
	// 2 Bad Value


// Control
#define m0Brake			0x86
#define m0Forward 		0x88
#define m0Reverse 		0x8A

#define m1Brake			0x87
#define m1Forward 		0x8C
#define m1Reverese		0x8E

// feedback
#define getM0Current	0x90
#define getM1Currnet 	0x91
#define getM0Speed		0x92
#define getM1Speed 		0x93

// constants
#define maxSpeed 		127
#define maxBrake 		127


// Motor driver commands (UART)
extern uint8_t const motor0[3];	// the left motor
extern uint8_t const motor1[3]; // the right motor
extern uint8_t rotation;
extern uint8_t rotationOld;
extern uint8_t speed;
extern uint8_t brake;


void MotorsBrake(UART_HandleTypeDef *huart, uint8_t *brake);
void MotorsOn(UART_HandleTypeDef *huart, uint8_t *speed, uint8_t rotation);

void TestDriver(UART_HandleTypeDef *huart);
void moveMotors(UART_HandleTypeDef *uart2, TIM_HandleTypeDef *tim5, uint8_t speed);
void usDelay(TIM_HandleTypeDef *tim5, uint32_t us);

#endif /* INC_QIK_2S12V10_LIB_H_ */






