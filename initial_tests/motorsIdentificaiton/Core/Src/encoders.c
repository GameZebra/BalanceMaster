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

float encoderLvalues[10];
float encoderRvalues[10];
float lSum=0, rSum=0, lSpeed=0, rSpeed=0;

float encoderTd = 0.002;

// filter
float b[4] = {0.0007, 0.0021, 0.0021, 0.0007};
float a[4] = {1.0, -2.6236, 2.3147, -0.6855};
IIR3_Filter lFilter, rFilter;


// debug
//int16_t encoderLSpeedMax = 0;
//int16_t encoderRSpeedMax = 0;
//int16_t encoderLSpeedMin = 0;
//int16_t encoderRSpeedMin = 0;


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

void filterEncodersMovingAverage(){
	  lSum = 0;
	  rSum = 0;
	  for(uint8_t i =9; i>0; i--){
		  encoderLvalues[i]=encoderLvalues[i-1];
		  lSum += encoderLvalues[i];
		  encoderRvalues[i]=encoderRvalues[i-1];
		  rSum += encoderRvalues[i];
	  }
	  encoderRvalues[0] = encoderRSpeed;
	  encoderLvalues[0] = encoderLSpeed;
	  lSum += encoderLSpeed;
	  rSum += encoderRSpeed;
	  lSpeed = lSum / 10.0;
	  rSpeed = rSum / 10.0;
}


