/*
 * encoders.h
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "stm32f4xx_hal.h"

extern int16_t encoderL;
extern int16_t encoderR;

extern int16_t encoderLOld;
extern int16_t encoderROld;
extern int16_t encoderLSpeed;
extern int16_t encoderRSpeed;


extern float encoderTd;

//debug values
extern int16_t encoderLSpeedMax;
extern int16_t encoderRSpeedMax;
extern int16_t encoderLSpeedMin;
extern int16_t encoderRSpeedMin;


void getEncoders(TIM_HandleTypeDef *T2, TIM_HandleTypeDef *T3);
#endif /* INC_ENCODERS_H_ */
