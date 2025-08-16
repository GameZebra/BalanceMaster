/*
 * filters.h
 *
 *  Created on: Aug 15, 2025
 *      Author: mrxmi
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "stm32f4xx_hal.h"

typedef struct {
    float b[4];  // numerator coefficients
    float a[4];  // denominator coefficients (a[0] = 1.0)
    float x[3];  // previous input samples
    float y[3];  // previous output samples
} IIR3_Filter;

void IIR3_Init(IIR3_Filter *f, float *b_coeffs, float *a_coeffs);
float IIR3_Process(IIR3_Filter *f, float input);
float movingAverage(float *values, float newValue, uint8_t order);

#endif /* INC_FILTERS_H_ */
