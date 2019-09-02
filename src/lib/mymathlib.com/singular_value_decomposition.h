
#ifndef SINGULAR_VALUE_DECOMPOSITION_H
#define SINGULAR_VALUE_DECOMPOSITION_H

#ifdef __cplusplus
  extern "C" {
#endif

int Singular_Value_Decomposition(double* A, int nrows, int ncols, double* U,
                      double* singular_values, double* V, double* dummy_array);


void Singular_Value_Decomposition_Solve(double* U, double* D, double* V,
                double tolerance, int nrows, int ncols, double *B, double* x);


void Singular_Value_Decomposition_Inverse(double* U, double* D, double* V,
                        double tolerance, int nrows, int ncols, double *Astar);

#endif
