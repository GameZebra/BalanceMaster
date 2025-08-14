/*
 * uart_debug.h
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#ifndef INC_UART_DEBUG_H_
#define INC_UART_DEBUG_H_

#include "stm32f4xx_hal.h"
#include "qik_2s12v10_lib.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "encoders.h"
#include "control.h"


void pcTransmitTime(UART_HandleTypeDef *uart5);

void pcTransmitAngles(UART_HandleTypeDef *uart5);

void pcTransmitEncoders(UART_HandleTypeDef *uart5);
void pcTransmitEncodersPositions(UART_HandleTypeDef *uart5);
void pcTransmitEncodersSpeeds(UART_HandleTypeDef *uart5);

void pcTransmitControll(UART_HandleTypeDef *uart5);

void pcTransmitAcc(UART_HandleTypeDef *uart5);

// and more eventually


void pcTransmitBin(UART_HandleTypeDef *uart5);


#endif /* INC_UART_DEBUG_H_ */
