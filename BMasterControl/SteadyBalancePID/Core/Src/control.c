/*
 * control.c
 *
 *  Created on: Jun 25, 2025
 *      Author: mrxmi
 */

#include "control.h"

// PID constants
float Kp =  3.0f;
float Ki =  0.0f;
float Kd = 2.0f;


// target
float setpoint = -0.5;
float setPointDelta = 0.00;
uint8_t isReady = 1;
float leftSetup[100];
float minD = 100;
float minDAngle[5] = {0, 0, 0, 0, 0};
int8_t sign = 1;
int8_t signOld = 1;
uint8_t signChanges = 0;
uint8_t moved = 0;
// State variables
float error = 0.0f;
float previous_error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float output = 0.0f;
float outputOld = 0.0f;
uint8_t dirChange = 0;

// debug
float dMAX = 0;
float lineFixed = 8;
float lineSpeed = 0.3;

// Sample time
float dt = 0.001f;  // 1 ms



void calculateSpeed(){
	error = setpoint - angle;
	integral += error * dt;
	Kp = (int)(lineSpeed*fabs(error))+lineFixed;
	derivative = (gyroAngle - previousAngle) / dt;
	if(derivative>127.0/Kd){
		derivative = 127;
	}
	output = Kp * error + Ki * integral + Kd * derivative;
	previous_error = error;
	previousAngle = gyroAngle;

	if (error<-0.005){
		rotation = 2;
	}
	else if (error>0.005){
		rotation = 0;
	}
	else{
		rotation = 1;
	}


	if (output<-127){
		output = -127;
	}
	else if (output>127){
		output = 127;
	}
	speed = abs((int)output);

	if (dMAX < derivative){
		dMAX = derivative;
	}
}



// target update eventually
