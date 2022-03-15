//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//
#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "InverseKinematicsWAM_types.h"

// Function Declarations
extern void xgemv(int m, int n, const double A[16], int ia0, const double x[16],
                  int ix0, double y[4]);

#endif

//
// File trailer for xgemv.h
//
// [EOF]
//
