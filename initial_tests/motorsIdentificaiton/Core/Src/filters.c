/*
 * filters.c
 *
 *  Created on: Aug 15, 2025
 *      Author: mrxmi
 */
#include "filters.h"

// Initialize filter state
void IIR3_Init(IIR3_Filter *f, float *b_coeffs, float *a_coeffs)
{
    for(int i=0; i<4; i++) {
        f->b[i] = b_coeffs[i];
        f->a[i] = a_coeffs[i];
    }
    for(int i=0; i<3; i++) {
        f->x[i] = 0.0f;
        f->y[i] = 0.0f;
    }
}

// Filter one sample
float IIR3_Process(IIR3_Filter *f, float input)
{
    // Compute output
    float output = f->b[0]*input
                 + f->b[1]*f->x[0]
                 + f->b[2]*f->x[1]
                 + f->b[3]*f->x[2]
                 - f->a[1]*f->y[0]
                 - f->a[2]*f->y[1]
                 - f->a[3]*f->y[2];

    // Shift previous inputs
    f->x[2] = f->x[1];
    f->x[1] = f->x[0];
    f->x[0] = input;

    // Shift previous outputs
    f->y[2] = f->y[1];
    f->y[1] = f->y[0];
    f->y[0] = output;

    return output;
}



