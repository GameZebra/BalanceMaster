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


void MotorsBrake(UART_HandleTypeDef *uart2, uint8_t *brake){
	HAL_UART_Transmit(uart2, &motor0[1], 1, 20);
	HAL_UART_Transmit(uart2, brake, 1, 20);
	HAL_UART_Transmit(uart2, &motor1[1], 1, 20);
	HAL_UART_Transmit(uart2, brake, 1, 20);
}

void MotorsOn(UART_HandleTypeDef *uart2, uint8_t *speed, uint8_t rotation){
	HAL_UART_Transmit(uart2, &motor0[rotation], 1, 20);
	HAL_UART_Transmit(uart2, speed, 1, 20);
	HAL_UART_Transmit(uart2, &motor1[rotation], 1, 20);
	HAL_UART_Transmit(uart2, speed, 1, 20);
}


void TestDriver(UART_HandleTypeDef *huart){
	  uint8_t testSpeed = 127;
	  MotorsOn(huart, &testSpeed, 0);
	  HAL_Delay(300);
	  MotorsBrake(huart, &testSpeed);
	  HAL_Delay(1500);
}

void moveMotors(UART_HandleTypeDef *uart2, TIM_HandleTypeDef *tim5, uint8_t speed){
	 if(rotation != rotationOld){
		 dirChange = 1;
		 moved =0;
		 rotationOld = rotation;
	 }
	 else{
		 //dirChange = 0;
		 if(fabs(error) < 0.2){
			 moved++;
		 }

	 }
	 if(dirChange==1){
		 if(abs(encoderRSpeed) > 1800){
			MotorsBrake(uart2, brake);
			usDelay(tim5, 400);
		 }
		 //else{
			 dirChange = 0;
		// }
	 }
	if(dirChange == 0){
		MotorsOn(uart2, &speed, rotation);
	}

}


void usDelay(TIM_HandleTypeDef *tim5, uint32_t us) {

    volatile uint32_t *cnt = tim5->Instance->CNT;

    __HAL_TIM_SET_COUNTER(tim5, us);
	// Set the counter to number of us
    HAL_TIM_Base_Start(tim5);        // Fire up the timer
    while (*cnt != 0);                 // Just wait until 0
    HAL_TIM_Base_Stop(tim5);

}
