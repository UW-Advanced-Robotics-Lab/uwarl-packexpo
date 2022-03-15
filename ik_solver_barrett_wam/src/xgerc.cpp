//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgerc.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xgerc.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                double alpha1
//                int ix0
//                const double y[4]
//                double A[16]
//                int ia0
// Return Type  : void
//
void xgerc(int m, int n, double alpha1, int ix0, const double y[4], double A[16],
           int ia0)
{
  int jA;
  int jy;
  int j;
  double temp;
  int ix;
  int i4;
  int ijA;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 1; j <= n; j++) {
      if (y[jy] != 0.0) {
        temp = y[jy] * alpha1;
        ix = ix0;
        i4 = m + jA;
        for (ijA = jA; ijA < i4; ijA++) {
          A[ijA] += A[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += 4;
    }
  }
}

//
// File trailer for xgerc.cpp
//
// [EOF]
//
