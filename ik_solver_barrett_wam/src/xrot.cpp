//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xrot.h"

// Function Definitions

//
// Arguments    : int n
//                double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(int n, double x[16], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  if (!(n < 1)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 1; k <= n; k++) {
      temp = c * x[ix] + s * x[iy];
      x[iy] = c * x[iy] - s * x[ix];
      x[ix] = temp;
      iy += 4;
      ix += 4;
    }
  }
}

//
// File trailer for xrot.cpp
//
// [EOF]
//
