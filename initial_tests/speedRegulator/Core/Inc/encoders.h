/*
 * encoders.h
 *
 *  Created on: Aug 5, 2025
 *      Author: mrxmi
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "stm32f4xx_hal.h"

extern uint16_t encoderL;
extern uint16_t encoderR;

extern uint16_t encoderLOld;
extern uint16_t encoderROld;
extern float encoderLSpeed;
extern float encoderRSpeed;
#define impConst 0.08844

extern float encoderTd;

//debug values
extern int16_t encoderLSpeedMax;
extern int16_t encoderRSpeedMax;
extern int16_t encoderLSpeedMin;
extern int16_t encoderRSpeedMin;


void getEncoders(TIM_HandleTypeDef *T2, TIM_HandleTypeDef *T3);
#endif /* INC_ENCODERS_H_ */
