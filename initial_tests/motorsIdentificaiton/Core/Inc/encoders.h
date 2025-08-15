/*
 * encoders.h
 *
 *  Created on: Aug 5, 2025
 *      Author: mrxmi
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "stm32f4xx_hal.h"
#include "filters.h"

#define impConst 0.08844 // mm/impulse

extern uint16_t encoderL;
extern uint16_t encoderR;

extern uint16_t encoderLOld;
extern uint16_t encoderROld;
extern float encoderLSpeed;
extern float encoderRSpeed;

extern float encoderLvalues[10];
extern float encoderRvalues[10];
extern float lSum, rSum, lSpeed, rSpeed;

extern float encoderTd;

// filter
extern float b[4];
extern float a[4];
extern IIR3_Filter lFilter, rFilter;

//debug values
//extern int16_t encoderLSpeedMax;
//extern int16_t encoderRSpeedMax;
//extern int16_t encoderLSpeedMin;
//extern int16_t encoderRSpeedMin;


void getEncoders(TIM_HandleTypeDef *T2, TIM_HandleTypeDef *T3);
void filterEncodersMovingAverage();


#endif /* INC_ENCODERS_H_ */
