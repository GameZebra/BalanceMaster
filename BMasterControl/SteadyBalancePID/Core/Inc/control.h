/*
 * control.h
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_


#include "stm32f4xx_hal.h"
#include "qik_2s12v10_lib.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "math.h"

// PID constants
extern float Kp;
extern float Ki;
extern float Kd;


// target
extern float setpoint;
extern float setPointDelta;
extern uint8_t isReady;
extern float leftSetup[100];
extern float minD;
extern float minDAngle[5];
extern int8_t sign;
extern int8_t signOld;
extern uint8_t signChanges;
extern uint8_t moved;

// State variables
extern float error;
extern float previous_error;
extern float integral;
extern float derivative;
extern float output;
extern float outputOld;
extern uint8_t dirChange;
extern float dMAX;
extern float lineFixed;
extern float lineSpeed;

// Sample time
extern float dt;  // 1 ms


void calculateSpeed();


#endif /* INC_CONTROL_H_ */
