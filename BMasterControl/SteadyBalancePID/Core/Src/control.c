/*
 * control.c
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#include "control.h"





// PID constants
// 0 for the fastest pid; 1 fpr  the middle; 2 for slowest

// fastest pid
float Kp0 = 0.2, Ki0 = 1, Kd0 = 0.0;

// middle pid
float Kp1 = 195.0f, Kp1_slope = 40;
float Ki1 = 0.0f;
float Kd1 = 1.0f;

//slowest pid
float Kp2 = 0.001, Ki2 = 0.00006, Kd2 = 0.0001;


// target

//float setPointDelta = 0.00;
//uint8_t isReady = 1;
//float leftSetup[100];
//float minD = 100;
//float minDAngle[5] = {0, 0, 0, 0, 0};
//int8_t sign = 1;
//int8_t signOld = 1;
//uint8_t signChanges = 0;
//uint8_t moved = 0;

// State variables
// fast pid
float integralL = 0, integralR = 0, previousLSpeed= 0, previousRSpeed=0;
int8_t leftCtrl = 0, rightCtrl = 0;

// middle pid
//float error = 0.0f;
float previousFilteredAngle = 0.0f;
float integralAngle = 0.0f;
//float derivative = 0.0f;
float controlSpeed = 0.0f;
//float outputOld = 0.0f;
//uint8_t dirChange = 0;

// slow pid
float maxTargetAngle = 1.5;
float integralPosition = 0.0f;
float previousSetAngle = 0.0f;
float targetAngle = -0.5;				// output
uint16_t targetPosition = START_POSITION;		// target
uint16_t currentPosition = START_POSITION;		// input


// other control variables


// Sample time
float angleTd = 0.010f;  	// 10 ms
float positionTd = 0.010f;	// 10 ms

// debug
//float dMAX = 0;
//float lineFixed = 8;
//float lineSpeed = 0.3;
uint8_t controlMax = 60;



float calculateTargetSpeed(float setpointAngle, float measuredAngle, float measuredAVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt){ // ma not need rotation
	float error = fabs(setpointAngle - measuredAngle);
//	generalLimit(&error, 3.0);
	float Kp_dynamic = error*error* Kp1_slope + Kp;
//	float Kp_dynamic = error* Kp1_slope + Kp;
//	float Kp_dynamic = fabs(measuredAngle)* Kp1_slope + Kp;
//	float Kp_dynamic = measuredAngle*measuredAngle* Kp1_slope + Kp;

	controlSpeed = PID2(setpointAngle, measuredAngle, measuredAVelocity, Kp_dynamic, Ki, Kd, integral, previousMeasurment, dt);
	generalLimit(&controlSpeed, 1000);
	return controlSpeed;
}

float PID3(uint16_t setpoint, uint16_t measured, float measuredVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt){
	float error = setpoint - measured;
	*integral += error * dt;
	float derivative = measuredVelocity;
	float control = Kp * error + Ki * *integral + Kd * derivative;
	*previousMeasurment = measured;

	return control;
}

float PID2(float setpoint, float measuredAngle, float measuredAVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt){
	float error = setpoint - measuredAngle;
	*integral += error * dt;
	float derivative = measuredAVelocity;
	float control = Kp * error + Ki * *integral + Kd * derivative;
	*previousMeasurment = measuredAngle;

	return control;
}


float PID(float setpoint, float measured, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt){
	float error = setpoint - measured;
	*integral += error * dt;
	float derivative = (measured - *previousMeasurment)/dt;
	float control = Kp * error + Ki * *integral + Kd * derivative;
	*previousMeasurment = measured;

	return control;
}

uint8_t direction(float *speed, float histeresis){
	uint8_t rotation = 0;
	if (*speed< -histeresis){
		rotation = 2;
	}
	else if (*speed> histeresis){
		rotation = 0;
	}
	else{
		*speed = 0;
	}
	return rotation;
}

void controlLimit(float *control){
	if (*control<-127){
		*control = -127;
	}
	else if (*control>127){
		*control = 127;
	}
}

void generalLimit(float *control, float limit){
	if (*control<-limit){
		*control = -limit;
	}
	else if (*control>limit){
		*control = limit;
	}
}

int8_t motorControl(float setSpeed, float measuredSpeed, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt, uint8_t *rotation){
	float control = PID(setSpeed, measuredSpeed, Kp, Ki, Kd, integral, previousMeasurment, dt);
	*rotation = direction(&control, 0);
//	controlLimit(&control);			// driver limits
	generalLimit(&control, controlMax);
	generalLimit(integral, controlMax/Ki); // anti-windup
	control = fabs(control);
	return (int8_t)control;
}


// target update eventually
float CalculateTargetAngle(uint16_t setPosition, uint16_t measuredPosition, float measuredVelocity, float Kp, float Ki, float Kd, float *integral, float *previousMeasurment, float dt){
	float control = PID3(setPosition, measuredPosition, measuredVelocity, Kp, Ki, Kd, integral, previousMeasurment, dt);
	generalLimit(integral, maxTargetAngle/Ki);	// anti-windup
	generalLimit(&control, maxTargetAngle);		// not to fall too hard
	return -control;
}
