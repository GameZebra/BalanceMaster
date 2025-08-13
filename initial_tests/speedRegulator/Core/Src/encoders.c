/*
 * encoders.c
 *
 *  Created on: Aug 5, 2025
 *      Author: mrxmi
 */


#include "encoders.h"

uint16_t encoderL = 0;
uint16_t encoderR = 0;

uint16_t encoderLOld = 0;
uint16_t encoderROld = 0;
float encoderLSpeed = 0;
float encoderRSpeed = 0;


float encoderTd = 0.002;

int16_t encoderLSpeedMax = 0;
int16_t encoderRSpeedMax = 0;
int16_t encoderLSpeedMin = 0;
int16_t encoderRSpeedMin = 0;


void getEncoders(TIM_HandleTypeDef *T2, TIM_HandleTypeDef *T3){
	  encoderL = __HAL_TIM_GET_COUNTER(T2);
	  encoderR = __HAL_TIM_GET_COUNTER(T3);

	  encoderLSpeed = ((float)((int16_t)(encoderL-encoderLOld))/encoderTd) * impConst; // mm/sec
	  encoderLOld = encoderL;

	  encoderRSpeed = ((float)((int16_t)(encoderR-encoderROld))/encoderTd) * impConst; // mm/sec
	  encoderROld = encoderR;


	  // debug
	  /*
	  if (encoderLSpeed > encoderLSpeedMax){
	 	  encoderLSpeedMax = encoderLSpeed;
	   }else if(encoderLSpeed < encoderLSpeedMin){
	 	  encoderLSpeedMin = encoderLSpeed;
	   }

	   if (encoderRSpeed > encoderRSpeedMax){
	 	  encoderRSpeedMax = encoderRSpeed;
	   }else if(encoderRSpeed < encoderRSpeedMin){
	 	  encoderRSpeedMin = encoderRSpeed;
	   }
	   */
}

