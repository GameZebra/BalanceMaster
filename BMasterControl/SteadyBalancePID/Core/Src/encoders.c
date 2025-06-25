/*
 * encoders.c
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#include "encoders.h"

int16_t encoderL = 0;
int16_t encoderR = 0;

int16_t encoderLOld = 0;
int16_t encoderROld = 0;
int16_t encoderLSpeed = 0;
int16_t encoderRSpeed = 0;


float encoderTd = 0.001;

int16_t encoderLSpeedMax = 0;
int16_t encoderRSpeedMax = 0;
int16_t encoderLSpeedMin = 0;
int16_t encoderRSpeedMin = 0;


void getEncoders(TIM_HandleTypeDef *T2, TIM_HandleTypeDef *T3){
	  encoderL = __HAL_TIM_GET_COUNTER(T2);
	  encoderR = __HAL_TIM_GET_COUNTER(T3);

	  encoderLSpeed = (encoderL-encoderLOld)/encoderTd;
	  encoderLOld = encoderL;

	  encoderRSpeed = (encoderR-encoderROld)/encoderTd;
	  encoderROld = encoderR;


	  // debug
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
}
