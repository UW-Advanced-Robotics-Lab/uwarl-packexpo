//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cross.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//

// Include Files
#include "rt_nonfinite.h"
#include "KnobOperationWAM.h"
#include "cross.h"

// Function Definitions

//
// Arguments    : const double a[3]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// File trailer for cross.cpp
//
// [EOF]
//
