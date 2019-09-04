
#ifndef SINGULAR_VALUE_DECOMPOSITION_H
#define SINGULAR_VALUE_DECOMPOSITION_H

#ifdef __cplusplus
  extern "C" {
#endif

int Singular_Value_Decomposition(float* A, int nrows, int ncols, float* U,
                      float* singular_values, float* V, float* dummy_array);


void Singular_Value_Decomposition_Solve(float* U, float* D, float* V,
                float tolerance, int nrows, int ncols, float *B, float* x);


void Singular_Value_Decomposition_Inverse(float* U, float* D, float* V,
                        float tolerance, int nrows, int ncols, float *Astar);

#endif
