
#ifndef SINGULAR_VALUE_DECOMPOSITION_H
#define SINGULAR_VALUE_DECOMPOSITION_H

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>

int8_t Singular_Value_Decomposition(float* A, int8_t nrows, int8_t ncols, float* U,
                      float* singular_values, float* V, float* dummy_array);


void Singular_Value_Decomposition_Solve(float* U, float* D, float* V,
                float tolerance, int8_t nrows, int8_t ncols, float *B, float* x);


void Singular_Value_Decomposition_Inverse(float* U, float* D, float* V,
                        float tolerance, int8_t nrows, int8_t ncols, float *Astar);

#endif
