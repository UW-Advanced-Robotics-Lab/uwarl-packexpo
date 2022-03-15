//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xgemv.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                const double A[16]
//                int ia0
//                const double x[16]
//                int ix0
//                double y[4]
// Return Type  : void
//
void xgemv(int m, int n, const double A[16], int ia0, const double x[16], int
           ix0, double y[4])
{
  int iy;
  int i2;
  int iac;
  int ix;
  double c;
  int i3;
  int ia;
  if (n != 0) {
    for (iy = 1; iy <= n; iy++) {
      y[iy - 1] = 0.0;
    }

    iy = 0;
    i2 = ia0 + ((n - 1) << 2);
    for (iac = ia0; iac <= i2; iac += 4) {
      ix = ix0;
      c = 0.0;
      i3 = (iac + m) - 1;
      for (ia = iac; ia <= i3; ia++) {
        c += A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[iy] += c;
      iy++;
    }
  }
}

//
// File trailer for xgemv.cpp
//
// [EOF]
//
