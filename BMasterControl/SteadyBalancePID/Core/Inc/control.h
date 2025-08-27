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

#define START_POSITION 30000


// PID constants
// 0 - fastest; 1 - middle; 2 - slowest
extern float Kp0, Kp1, Kp2, Kp1_slope;
extern float Ki0, Ki1, Ki2;
extern float Kd0, Kd1, Kd2;


// target
//extern float setPointDelta;
//extern uint8_t isReady;
//extern float leftSetup[100];
//extern float minD;
//extern float minDAngle[5];
//extern int8_t sign;
//extern int8_t signOld;
//extern uint8_t signChanges;
//extern uint8_t moved;

// State variables
// fastest PID
extern float integralL, integralR, previousLSpeed, previousRSpeed;
extern int8_t leftCtrl, rightCtrl;

//extern float error;
extern float previousFilteredAngle;
extern float integralAngle;
//extern float derivative;
extern float controlSpeed;
//extern float outputOld;
//extern uint8_t dirChange;

// slow PID
// slow pid
extern float maxTargetAngle;
extern float integralPosition;
extern float previousSetAngle;
extern float targetAngle;			// output
extern uint16_t targetPosition;		// target
extern uint16_t currentPosition;	// input

// other control variables



// Sample time
extern float angleTd;  // 10 ms
extern float positionTd;


// debug
//extern float dMAX;
//extern float lineFixed;
//extern float lineSpeed;


float calculateSpeed(float setpointAngle, float measuredAngle, float measuredAVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt);
float PID3(uint16_t setpoint, uint16_t measured, float measuredVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt);
float PID2(float setpoint, float measuredAngle, float measuredAVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt);
float PID(float setpoint, float measured, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt);
uint8_t direction(float *speed, float histeresis);
void controlLimit(float *control);
void generalLimit(float *control, float limit);
int8_t motorControl(float setpoint, float measured, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt, uint8_t *rotation);

float CalculateTargetAngle(uint16_t setPosition, uint16_t measuredPosition, float measuredVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt);

#endif /* INC_CONTROL_H_ */
