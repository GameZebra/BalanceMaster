/*
 * qik_2s12v10_lib.c
 *
 *  Created on: Jun 5, 2025
 *      Author: mrxmi
 */
#include "qik_2s12v10_lib.h"


uint8_t const motor0[3] = {m0Forward, m0Brake, m0Reverse};	// the left motor
uint8_t const motor1[3] = {m1Reverese, m1Brake, m1Forward}; // the right motor
uint8_t rotation = 0;
uint8_t rotationOld = 0;
uint8_t speed = 0;
uint8_t brake = 127;


